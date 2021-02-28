/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2004  Rob O'Donnell
Copyright (C) 2005  Mike Wyatt

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public
License along with this program; if not, write to the Free
Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
Boston, MA  02110-1301, USA.
****************************************************************/

// Econet support for BeebEm
// Rob O'Donnell. robert@irrelevant.com, December 28th 2004.
// Mike Wyatt - further development, Dec 2005
// AUN by Rob Jun/Jul 2009


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "main.h"
#include "beebwin.h"
#include "econet.h"
#include "debug.h"
#include "6502core.h"
#include "sysvia.h"

typedef int				SOCKET;
#define SOCKET_ERROR	-1
#define INVALID_SOCKET	-1

// Station Configuration settings:
// You specify station number on command line.
// This allows multiple different instances of the emulator to be run and
// to communicate with each other.  Note that you STILL need to have them
// all listed in econet.cfg so each one knows where the other area.
// These, among others, are overridden in econet.cfg (See ReadNetwork)  )
bool confAUNmode = false; // Use AUN style Networking
bool confLEARN = false;   // Add receipts from unknow hosts to network table
bool confSTRICT = false;  // Assume network ip=stn number when sending to unknown hosts
bool confSingleSocket = true; // Use same socket for Send and receive
unsigned int FourWayStateTimeout  = 500000;
bool MassageNetworks = false; // massage network numbers on send/receive (add/sub 128)

int inmask, outmask;

bool EconetStateChanged = false;
char EconetEnabled;							// Enable hardware
bool EconetNMIenabled;						// 68B54 -> NMI enabled. (IC97)
int EconetTrigger;							// Poll timer

const unsigned char powers[4]={1,2,4,8};

// Frequency between network actions.
// max 250Khz network clock. 2MHz system clock. one click every 8 cycles.
// say one byte takes about 8 clocks, receive a byte every 64 cpu cycyes. ?
// (The reason for "about" 8 clocks is that as this a continuous syncronous tx,
// there are no start/stop bits, however to avoid detecting a dead line as ffffff
// zeros are added and removed transparently if you get more than five "1"s 
// during data transmission - more than 5 are flags or errors)
// 6854 datasheet has max clock frequency of 1.5MHz for the B version.
// 64 cycles seems to be a bit fast for 'netmon' prog to keep up - set to 128.

// #define TIMEBETWEENBYTES (128)
unsigned int TIMEBETWEENBYTES = 128;

// Station Configuration settings:
// You specify station number on command line.
// This allows multiple different instances of the emulator to be run and
// to communicate with each other.  Note that you STILL need to have them
// all listed in econet.cfg so each one knows where the other area.
// These, among others, are overridden in econet.cfg (See ReadNetwork)  )
unsigned char EconetStationNumber=0;	// default Station Number
unsigned int EconetListenPort=0;		// default Listen port
unsigned long EconetListenIP = 0x0100007f;

// IP settings:
SOCKET ListenSocket = SOCKET_ERROR;		// Listen socket
SOCKET RxSocket = SOCKET_ERROR;		// current receiving socket
SOCKET SendSocket;
bool ReceiverSocketsOpen = FALSE;		// used to flag line up and clock running

// Written in 2004:
// we will be using Econet over Ethernet as per AUN,
// however I've not got a real acorn ethernet machine to see how 
// it actually works!  The only details I can find is it is:
// "standard econet encpsulated in UDP to port 32768" and that
// Addressing defaults to "1.0.net.stn" where net >= 128 for ethernet.
// but can be overridden, so we won't worry about that.

// 2009: Now I come back to this, I know the format ... :-)
// and sure enough, it's pretty simple.
// It's translating the different protocols that was harder..
struct aunhdr
{
    unsigned char type;         /* AUN magic protocol byte */
#define AUN_TYPE_BROADCAST  1  // 'type' from netBSD aund.h
#define AUN_TYPE_UNICAST    2
#define AUN_TYPE_ACK        3 
#define AUN_TYPE_NACK       4
#define AUN_TYPE_IMMEDIATE  5 
#define AUN_TYPE_IMM_REPLY  6
    unsigned char port;     // dest port 
    unsigned char cb;       // flag 
    unsigned char pad;      // retrans 
    unsigned long handle;   // 4 byte sequence little-endian.
};

#define EC_PORT_FS              0x99
#define EC_PORT_PS_STATUS_ENQ   0x9f
#define EC_PORT_PS_STATUS_REPLY 0x9e
#define EC_PORT_PS_JOB          0xd1

unsigned long ec_sequence = 0;

unsigned int fourwaystage;
#define FWS_IDLE 0
#define FWS_SCOUTSENT 1
#define FWS_SCACKRCVD 2 
#define FWS_DATASENT  3
#define FWS_WAIT4IDLE 4
#define FWS_SCOUTRCVD 11
#define FWS_SCACKSENT 12
#define FWS_DATARCVD 13
#define FWS_IMMSENT 7
#define FWS_IMMRCVD 8


struct shorteconethdr
{
	unsigned char deststn;
	unsigned char destnet;
	unsigned char srcstn;
	unsigned char srcnet;
};
struct longeconetpacket
{
	unsigned char deststn;
	unsigned char destnet;
	unsigned char srcstn;
	unsigned char srcnet;
	unsigned char cb;
	unsigned char port;
//	unsigned char buff[2];
};

    
// MC6854 has 3 byte FIFOs. There is no wait for an end of data
// before transmission starts. Data is sent immediately it's put into
// the first slot.

// Does econet send multiple packets for big transfers, or just one huge 
// packet?
// What's MTU on econet? Depends on clock speed but its big (e.g. 100K).
// As we are using UDP, we will construct a 2048 byte buffer accept data
// into this, and send it periodically.  We will accept incoming data
// similarly, and dribble it back into the emulated 68B54.
// We should thus never suffer underrun errors....
// --we do actually flag an underrun, if data exceeds the size of the buffer.
// -- sniffed AUN between live arcs seems to max out at 1288 bytes (1280+header)
// --- bigger packets ARE possible - UDP fragments & reassembles transparently.. doh..

// 64K max.. can't see any transfers being neeeded larger than this too often!
// (and that's certainly larger than acorn bridges can cope with.)
#define ETHERNETBUFFERSIZE 65536

struct ethernetpacket
{
	union {
		unsigned char raw[8];
		aunhdr ah;
	} ;
	union {
		unsigned char buff[ETHERNETBUFFERSIZE];
		shorteconethdr eh;
	};
	volatile unsigned int Pointer;
	volatile unsigned int BytesInBuffer;
	unsigned long inet_addr;
	unsigned int port;
	unsigned int deststn;
	unsigned int destnet;
};


// buffers used to construct packets for sending out via UDP
ethernetpacket EconetRx;
ethernetpacket EconetTx;


// buffers used to construct packets sent to/received from bbc micro


struct econetpacket
{
	union {
		longeconetpacket eh;
		unsigned char buff[ETHERNETBUFFERSIZE+12];
	};
	volatile unsigned int Pointer;
	volatile unsigned int BytesInBuffer;
};

econetpacket BeebTx;
econetpacket BeebRx;

unsigned char BeebTxCopy[6]; // size of longeconetpacket structure

//unsigned char BeebRx.buff[2048];				// probably can't be bigger than 1500
//unsigned char BeebTx.buff[2048];				// (or 1458 on my lan)  sniffed packets max 1280+hdr
//volatile unsigned int BeebRx.Pointer=0;	// pointers etc
//volatile unsigned int BeebRx.BytesInBuffer=0;
//unsigned int BeebTx.Pointer=0;
//unsigned int BeebTx.BytesInBuffer=0;

// Holds data from econet.cfg file
struct ECOLAN {							// what we we need to find a beeb?
	unsigned char station;
	unsigned char network;
	unsigned long inet_addr;
	unsigned int port;
};

struct AUNTAB {
    unsigned long inet_addr;
    unsigned char network;
};

#define NETWORKTABLELENGTH 512          // total number of hosts we can know about
#define AUNTABLELENGTH 128              // number of disparate network in AUNMap
ECOLAN network[NETWORKTABLELENGTH];		// list of my friends! :-)
AUNTAB aunnet[AUNTABLELENGTH];          // AUNMap file guess mode.

unsigned int networkp = 0;				// how many friends do I have?
unsigned int aunnetp = 0;               // how many networks do I know about
unsigned int myaunnet = 0;              // aunnet table entry that I match. Should be -1 as 0 is valid.

char EconetCfgPath[512];				// where's my list saved?

unsigned char irqcause;					// flagto indicate cause of irq sr1b7
unsigned char sr1b2cause;				// flagto indicate cause of irq sr1b2

// A receiving station goes into flag fill mode while it is processing 
// a message.  This stops other stations sending messages that may interfere
// with the four-way handshake.  Attempting to notify evey station using 
// IP messages when flag fill goes active/inactive would be complicated and 
// would suffer from timing issues due to network latency, so a pseudo
// flag fill algorithm is emulated.  We assume that the receiving station 
// will go into flag fill when we send a message or when we see a message 
// destined for another station.  We cancel flag fill when we receive a 
// message as the other station must have cancelled flag fill.  In order to
// cancel flag fill after the last message of a four-way handshake we time it
// out - which is not ideal as we do not want to delay new messages any
// longer that we have to - but it will have to do for now!
bool FlagFillActive;                    // Flag fill state
int EconetFlagFillTimeoutTrigger;       // Trigger point for flag fill
int EconetFlagFillTimeout = 500000;     // Cycles for flag fill timeout
                                        // added cfg file to override this
int EconetSCACKtrigger;                 // trigger point for scout ack
int EconetSCACKtimeout = 500;           // cycles to delay before sending ack to scout (aun mode only)
int Econet4Wtrigger;
// Device and temp copy!
volatile MC6854 ADLC;
MC6854 ADLCtemp;

void closesocket(SOCKET socket)
{
	close(socket);
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void EconetReset(void) {
	char info[200];

	if (DebugEnabled) {
		DebugDisplayTrace(DEBUG_ECONET, true, "Econet: Reset");
		if (EconetEnabled) {
			DebugDisplayTrace(DEBUG_ECONET, true, "Econet: hardware enabled");
		} else {
			DebugDisplayTrace(DEBUG_ECONET, true, "Econet: hardware disabled");
		}
	}

    // hardware operations:
	// set RxReset and TxReset
	ADLC.control1 = 192;
	// reset TxAbort, RTS, LoopMode, DTR
	ADLC.control4 = 0; //ADLC.control4 & 223;
	ADLC.control2 = 0; //ADLC.control2 & 127;
	ADLC.control3 = 0; //ADLC.control3 & 95;

	// clear all status conditions
	ADLC.status1=0;	//cts - clear to send line input (no collissions talking udp)
	ADLC.status2=0;	//dcd - no clock (until sockets initialised and open)
	ADLC.sr2pse=0;

	//software stuff:
	EconetRx.Pointer = 0;
	EconetRx.BytesInBuffer = 0;
	EconetTx.Pointer = 0;
	EconetTx.BytesInBuffer = 0;

    BeebRx.Pointer = 0;
    BeebRx.BytesInBuffer = 0;
    BeebTx.Pointer = 0;
    BeebTx.BytesInBuffer = 0;

    fourwaystage = FWS_IDLE;        // used for AUN mode translation stage.

	ADLC.rxfptr = 0;
	ADLC.rxap = 0;
	ADLC.rxffc = 0;
	ADLC.txfptr = 0;
	ADLC.txftl = 0;
	
	ADLC.idle = 1;
	ADLC.cts = 0;

	irqcause = 0;
	sr1b2cause = 0;

	FlagFillActive = false;
	EconetFlagFillTimeoutTrigger = 0;

	// kill anything that was in use
	if (ReceiverSocketsOpen) {
		if (!confSingleSocket) closesocket(SendSocket);
		closesocket(ListenSocket);
		ReceiverSocketsOpen = false;
	}

	// Stop here if not enabled
	if (!EconetEnabled)
		return;

	// Read in econet.cfg.  Done here so can refresh it on Break.
	ReadNetwork();

	//----------------------
	// Create a SOCKET for listening for incoming connection requests.
	ListenSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (ListenSocket == INVALID_SOCKET) {
		sprintf(info, "Econet: Failed to open listening socket");
		EconetError(info);
		return;
	}

	//----------------------
	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	sockaddr_in service;
	bzero(&service, sizeof(service));
	service.sin_family = AF_INET;
	service.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("127.0.0.1");

	// Already have a station num? Either from command line or a free one 
	// we found on previous reset.
	if (EconetStationNumber != 0) {
		// Look up our port number in network config
		for (unsigned int i = 0; i < networkp; ++i) {
			if (network[i].station == EconetStationNumber) {
				EconetListenPort = network[i].port;
                EconetListenIP = network[i].inet_addr;
				break;
			}
		}
		if (EconetListenPort != 0) {
			service.sin_port = htons(EconetListenPort);
            service.sin_addr.s_addr = EconetListenIP;
			if (bind(ListenSocket, (struct sockaddr*)&service, sizeof(service)) == SOCKET_ERROR) {
				if (errno == EADDRINUSE)
				{
				}
				sprintf(info, "Econet: Failed to bind to port %d (error %d)", EconetListenPort, errno);
				EconetError(info);
				closesocket(ListenSocket);
				return;
			}
		} else {
			sprintf(info, "Econet: Failed to find station %d in econet.cfg", EconetStationNumber);
			EconetError(info);
			return;
		}

	} else {
		// Station number not specified, find first one not already in use.
		char localhost[256];
		hostent *hent;
		struct in_addr localaddr;

		// Get localhost IP address
		if (gethostname(localhost, 256) != SOCKET_ERROR &&
			  (hent = gethostbyname(localhost)) != NULL) {

			// see if configured addresses match local IPs
			for (unsigned int i = 0; i < networkp && EconetStationNumber == 0; ++i) {
			
				// Check address for each network interface/card
				for (int a = 0; hent->h_addr_list[a] != NULL && EconetStationNumber == 0; ++a) {
					memcpy(&localaddr, hent->h_addr_list[a], sizeof(struct in_addr));
		
					if (network[i].inet_addr == inet_addr("127.0.0.1") ||
						network[i].inet_addr == localaddr.s_addr) {
						// network[i].inet_addr == localaddr.s_un.s_addr) {
						
						service.sin_port = htons(network[i].port);
                        service.sin_addr.s_addr = network[i].inet_addr;
						
						if (bind(ListenSocket, (sockaddr*)&service, sizeof(service)) == 0) {
							EconetListenPort = network[i].port;
                            EconetListenIP = network[i].inet_addr;
							EconetStationNumber = network[i].station;
						}
					}
				}
			}
				
			if (EconetListenPort == 0) {
                // Still can't find one ... strict mode ?

                if (confSTRICT && confAUNmode && networkp < NETWORKTABLELENGTH) {
					if (DebugEnabled)
						DebugDisplayTrace(DEBUG_ECONET, true, "Econet: No free hosts in table; trying automatic mode..");

					for (unsigned int j=0; j<aunnetp && EconetStationNumber == 0; j++) {
						for (int a = 0; hent->h_addr_list[a] != NULL && EconetStationNumber == 0; ++a) {
							memcpy(&localaddr, hent->h_addr_list[a], sizeof(struct in_addr));
						
							if (aunnet[j].inet_addr == (localaddr.s_addr & 0x00FFFFFF)) {
								service.sin_port = htons(32768);;
								service.sin_addr.s_addr = localaddr.s_addr;
								if (bind(ListenSocket, (sockaddr*)&service, sizeof(service)) == 0) {
									myaunnet = j;
									network[networkp].inet_addr = EconetListenIP = localaddr.s_addr;
									network[networkp].port = EconetListenPort = 32768;
									network[networkp].station = EconetStationNumber = localaddr.s_addr >>24;
									network[networkp].network = aunnet[j].network;
									networkp++;
								}
							}
						}
					}
				}

				if (EconetStationNumber == 0) {
					EconetError("Econet: Failed to find free station/port to bind to");
					return;
				}
			}
		} else {
			sprintf(info, "Econet: Failed to resolve local IP address");
			EconetError(info);
//			localaddr.s_addr = inet_addr("127.0.0.1");
			return;
		}

	}
	if (DebugEnabled) {
		sprintf(info, "Econet: Station number %d, port %d", EconetStationNumber, EconetListenPort);
		DebugDisplayTrace(DEBUG_ECONET, true, info);
	}

	// On Master the station number is read from CMOS so update it
	if (MachineType == 3) {
		CMOSWrite(0x0e, EconetStationNumber);
	}
		
	//---------------------
	// Socket used to send messages.
    if (confSingleSocket) {
        SendSocket = ListenSocket;
    } else {
	    SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	    if (SendSocket == INVALID_SOCKET) {
 	    	sprintf(info, "Econet: Failed to open sending socket");
		    EconetError(info);
		    closesocket(ListenSocket);
		    return;
	    }
    }

    // this call is what allows broadcast packets to be sent:
    char broadcast = '1';
    if (setsockopt(SendSocket, SOL_SOCKET, SO_BROADCAST, &broadcast,
        sizeof broadcast) == -1)  {
        sprintf(info, "Econet: Failed to set socket for broadcasts (error %d)", errno);
        EconetError(info);
        closesocket(ListenSocket);
        return;
    }

	ReceiverSocketsOpen = true;

	// how long before we bother with poll routine?
    SetTrigger(TIMEBETWEENBYTES,EconetTrigger);

	EconetStateChanged = true;
}

//-----------------------------------------------------------------------------
//---------------------------------------------------------------------------

void ReadNetwork(void) {
	// read econet.cfg file into network table
	// probably could be done easier or better.. it's knowing where to look to 
	// find out the functions....
	FILE *EcoCfg;
	char TmpPath[256];
	char value[80];
	char EcoNameBuf[256];
	char *EcoName=EcoNameBuf;
	char info[200];
	unsigned int i;
	unsigned int j;
	unsigned int p;
    unsigned int q;
	
	strcpy(TmpPath,EconetCfgPath);
	strcat(TmpPath,"econet.cfg");
	EcoCfg=fopen(TmpPath,"rt");
	if (EcoCfg==NULL) {
		sprintf(info, "Econet: Failed to open configuration file:\n  %s", TmpPath);
		EconetError(info);
		networkp = 0;
		network[0].station = 0;
	} else {
		networkp = 0;
		do {
			if (fgets(EcoName,255,EcoCfg) == NULL) break;
			if (DebugEnabled) {
				sprintf(info, "Econet: ConfigFile %s", EcoName);
				DebugDisplayTrace(DEBUG_ECONET, true, info);
			}
			if (EcoName[0] != '#') {
				i=0; p=0; q=0;
				do {
					j = 0;
					do {
						value[j] = EcoName[i];
						i++; j++;
					} while (EcoName[i] != ' ' && i < strlen(EcoName) && j<80);
					value[j] =0;
                    if (p==0) {
                        if (strcmp("AUNMODE", value) == 0) q = 1;
                        if (strcmp("LEARN", value) == 0) q = 2;
                        if (strcmp("AUNSTRICT", value) == 0) q = 3;
                        if (strcmp("SINGLESOCKET", value) == 0) q = 4;
                        if (strcmp("FLAGFILLTIMEOUT", value) == 0) q = 5;
                        if (strcmp("SCACKTIMEOUT", value) == 0) q = 6;
                        if (strcmp("TIMEBETWEENBYTES", value) == 0) q = 7;
                        if (strcmp("FOURWAYTIMEOUT", value) == 0) q = 8;
                        if (strcmp("MASSAGENETS", value) == 0) q = 9;
                        if (q == 0) network[networkp].network = atoi(value);
                    } 
                    if (p==1) {
                        if (q == 1) confAUNmode = (atoi(value) !=0);
                        if (q == 2) confLEARN = (atoi(value) != 0);
                        if (q == 3) confSTRICT = (atoi(value) != 0);
                        if (q == 4) confSingleSocket = (atoi(value) != 0);
                        if (q == 5) EconetFlagFillTimeout = (atoi(value));
                        if (q == 6) EconetSCACKtimeout = (atoi(value));
                        if (q == 7) TIMEBETWEENBYTES = (atoi(value));
                        if (q == 8) FourWayStateTimeout = (atoi(value));
                        if (q == 9) MassageNetworks = (atoi(value) != 0);
                        if (q == 0) network[networkp].station = atoi(value);
                        else {
                            if (DebugEnabled) {
                                sprintf(info, "Econet: Config option %i flags %s", q, value);
                                DebugDisplayTrace(DEBUG_ECONET, true, info);
                            }
                            break;
                        } 
                    }
					if (p==2) network[networkp].inet_addr = inet_addr(value);
					if (p==3) network[networkp].port = atoi(value);
					do i++; while (EcoName[i] == ' ' && i < strlen(EcoName));
					p++;
				} while (i < strlen(EcoName));
				if (DebugEnabled && q == 0) {
					sprintf(info, "Econet: ConfigFile Net %i Stn %i IP %08x Port %i",
							network[networkp].network, network[networkp].station,
							(unsigned int) network[networkp].inet_addr, network[networkp].port );
					DebugDisplayTrace(DEBUG_ECONET, true, info);
				}
				if (q == 0 && p == 4) networkp++;	// there were correct qty fields on line
				// otherwise pointer not incremented, next line overwrites it.
			}
		} while (networkp < NETWORKTABLELENGTH);
		network[networkp].station = 0;

        if (MassageNetworks) {
            inmask = 255;
            outmask = 0;
        } else {
            inmask = 127;
            outmask = 128;
        }

		fclose(EcoCfg);
	}

    aunnetp = 0;
    aunnet[0].network = 0;  // terminate table 

    if (confAUNmode) { // don't bother reading file if not using AUN.
		strcpy(TmpPath,EconetCfgPath);
		strcat(TmpPath,"AUNMap");
		EcoCfg=fopen(TmpPath,"rt");
		if (EcoCfg==NULL) {
			sprintf(info, "Econet: Failed to open configuration file:\n  %s", TmpPath);
			EconetError(info);
			aunnet[0].network = 0;
		} else {
			do {
				if (fgets(EcoName,255,EcoCfg) == NULL) break;
				if (DebugEnabled) {
					sprintf(info, "Econet: ConfigFile %s", EcoName);
					DebugDisplayTrace(DEBUG_ECONET,  true, info);
				}
				if (EcoName[0] != '#' && EcoName[0] != '|') {
					i=0; p=0; q=0;
					do {
						j = 0;
						do {
							value[j] = EcoName[i];
							i++; j++;
						} while (EcoName[i] != ' ' && i < strlen(EcoName) && j<80);
						value[j] =0;
						if (p==0) {
							if (strcmp("ADDMAP",value) == 0) q = 1;
						}
						if (p==1) {
							if (q == 1 ) aunnet[aunnetp].inet_addr = inet_addr(value) & 0x00FFFFFF; // stored as lsb..msb ?!?!
						}
						if (p==2) {
							if (q == 1 ) aunnet[aunnetp].network = (atoi(value) & inmask); //30jun strip b7
						}
						do i++; while (EcoName[i] == ' ' && i < strlen(EcoName));
						p++;
					} while (i < strlen(EcoName));
					if (q == 1 && p == 3) {
						if (DebugEnabled) {
							sprintf(info, "Econet: AUNMap Net %i IP %08lx ",
									aunnet[aunnetp].network, aunnet[aunnetp].inet_addr );
							DebugDisplayTrace(DEBUG_ECONET, true, info);
						}
						// note which network we are a part of.. this wont work on first run as listenip not set!
						if (aunnet[aunnetp].inet_addr == (EconetListenIP & 0x00FFFFFF)) {
							myaunnet = aunnetp;
							if (DebugEnabled) {
								DebugDisplayTrace(DEBUG_ECONET, true, "Econet: ..and that's the one we're in");
							}
						}

						aunnetp++;	// there were correct qty fields on line
					}
					// otherwise pointer not incremented, next line overwrites it.
				}
			} while (aunnetp < AUNTABLELENGTH);
			fclose(EcoCfg);
		}
	}

	aunnet[aunnetp].network = 0; // terminate table. 0 is always local so should not be in file.
} // end ReadNetwork


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// read to FE18..

unsigned char Read_Econet_Station(void) {
	if (DebugEnabled) {
		char info[200];
		sprintf(info, "Econet: Read Station %02X", (int)EconetStationNumber);
		DebugDisplayTrace(DEBUG_ECONET, true, info);
	}
	return(EconetStationNumber);
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// read to FEA0-3

unsigned char ReadEconetRegister(unsigned char Register) {
	if (DebugEnabled) {
		char info[200];
		sprintf(info, "Econet: Read ADLC %02X", (int)Register);
		DebugDisplayTrace(DEBUG_ECONET, true, info);
		debugADLCprint();
	}
	if (Register==0) {
		return (ADLC.status1);
	}
	if (Register==1) {
		return (ADLC.status2);
	}
	if (Register>1) { 
		if (((ADLC.control1 & 64)==0) && ADLC.rxfptr) {	// rxreset not set and someting in fifo
			if (DebugEnabled) {
				char info[200];
				sprintf(info, "Econet: Returned fifo: %02X", (int)ADLC.rxfifo[ADLC.rxfptr-1]);
				DebugDisplayTrace(DEBUG_ECONET, true, info);
				debugADLCprint();
			}
			
			if (ADLC.rxfptr) {
				EconetStateChanged = true;
				return (ADLC.rxfifo[--ADLC.rxfptr]);	// read rx buffer
			} else {
				return (0);
			}
		}
	}

	return 0; 
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
// write to FEA0-3

void WriteEconetRegister(unsigned char Register, unsigned char Value) {
	if (DebugEnabled) {
		char info[200];
		sprintf(info, "Econet: Write ADLC %02X = %02X", (int)Register, (int)Value);
		DebugDisplayTrace(DEBUG_ECONET, true, info);
	}
	
	// command registers are really just a set of flags that affect 
	// operation of the rest of the device.

	if (Register == 0) {								// adr 00
		ADLC.control1 = Value;
	} else if (Register == 1 && !(ADLC.control1 & 1)) {	// adr 01 & AC=0
		ADLC.control2 = Value;
	} else if (Register == 1 && (ADLC.control1 & 1)) {	// adr 01 & AC=1
		ADLC.control3 = Value;
	} else if (Register == 3 && (ADLC.control1 & 1)) {	// adr 03 & AC=1
		ADLC.control4 = Value;
	} else if (Register == 2 || Register == 3) {		// adr 02 or adr 03 & AC=0
		// cannot write an output byte if txreset is set
		// register 2 is an output byte
		// register 3 with c1b0=0 is output byte & finalise tx.
		// can also finalise tx by setting a control bit.so do that automatically for reg 3 
		// worry about actually sending stuff in the poll routines, not here.
		if ((ADLC.control1 & 128)==0) {
			ADLC.txfifo[2] = ADLC.txfifo[1];
			ADLC.txfifo[1] = ADLC.txfifo[0];
			ADLC.txfifo[0] = Value;
			ADLC.txfptr++; 
			ADLC.txftl = ADLC.txftl <<1;	///	shift txlast bits up.
			if (Register == 3) ADLC.control2 |= 16;	// set txlast control flag ourself
		}
	}
	
	if (DebugEnabled) debugADLCprint();

	EconetStateChanged = true;
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// Optimisation - only call real poll routine when something has changed

bool EconetPoll(void) {		//return NMI status
	if (EconetStateChanged || EconetTrigger<=TotalCycles) {
		EconetStateChanged = false;

		// Don't poll if failed to init sockets
		if (ReceiverSocketsOpen) {
			return EconetPoll_real();
		}
	}
	return false;
}


//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// Run when state changed or time to check comms.
// The majority of this code is to handle the status registers.
// These are just flags that depend on the tx & rx satus, and the control flags.
// These change immediately anything happens, so need refreshing all the time,
// as rx and tx operations can depend on them too.  It /might/ be possible to 
// only re-calculate them when needed (e.g. on a memory-read or in the receive
// routines before they are checked) but for the moment I just want to get this
// code actually working!

bool EconetPoll_real(void) {		//return NMI status
	char info[200];
	bool interruptnow = false;

	// save flags
	ADLCtemp.status1 = ADLC.status1;
	ADLCtemp.status2 = ADLC.status2;

	// okie dokie.  This is where the brunt of the ADLC emulation & network handling will happen.

	// look for control bit changes and take appropriate action

	// CR1b0 - Address Control - only used to select between register 2/3/4
	//		no action needed here
	// CR1b1 - RIE - Receiver Interrupt Enable - Flag to allow receiver section to create interrupt.
	//		no action needed here
	// CR1b2 - TIE - Transmitter Interrupt Enable - ditto
	//		no action needed here
	// CR1b3 - RDSR mode. When set, interrupts on received data are inhibited.
	//		unsupported - no action needed here
	// CR1b4 - TDSR mode. When set, interrupts on trasmit data are inhibited.
	//		unsupported - no action needed here
	// CR1b5 - Discontinue - when set, discontinue reception of incoming data.
	//	    automatically reset this when reach the end of current frame in progress
	//		automatically reset when frame aborted bvy receiving an abort flag, or DCD fails.
	if (ADLC.control1 & 32) {
		if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, "EconetPoll: RxABORT is set\0");
		EconetRx.Pointer =0;
		EconetRx.BytesInBuffer = 0;
		ADLC.rxfptr = 0;
		ADLC.rxap = 0;
		ADLC.rxffc = 0;
		ADLC.control1 &= ~32;	// reset flag
        fourwaystage = FWS_IDLE;
	}
	// CR1b6 - RxRs - Receiver reset. set by cpu or when reset line goes low.
	//		all receive operations blocked (bar dcd monitoring) when this is set.
	//		see CR2b5
	// CR1b7 - TxRS - Transmitter reset. set by cpu or when reset line goes low.
	//		all transmit operations blocked (bar cts monitoring) when this is set.
	//		no action needed here; watch this bit elsewhere to inhibit actions
			
	// -----------------------
	// CR2b0 - PSE - priotitised status enable - adjusts how status bits show up.
	//         See sr2pse and code in status section
	// CR2b1 - 2byte/1byte mode.  set to indicate 2 byte mode. see trda status bit.
	// CR2b2 - Flag/Mark idle select. What is transmitted when tx idle. ignored here as not needed
	// CR2b3 - FC/TDRA mode - does status bit SR1b6 indicate 1=frame complete, 
	//	0=tx data reg available. 1=frame tx complete.  see tdra status bit
	// CR2b4 - TxLast - byte just put into fifo was the last byte of a packet.
	if (ADLC.control2 & 16) {					// TxLast set
		ADLC.txftl |= 1;						//	set b0 - flag for fifo[0]
		ADLC.control2 &= ~16;					// clear flag.
	}

	// CR2b5 - CLR RxST - Clear Receiver Status - reset status bits
	if ((ADLC.control2 & 32) || (ADLC.control1 & 64)) {	// or rxreset
		ADLC.control2 &= ~32;					// clear this bit
		ADLC.status1 &= ~10;  					// clear sr2rq, FD
		ADLC.status2 &= ~126;					// clear FV, RxIdle, RxAbt, Err, OVRN, DCD

		if ((ADLC.control2 & 1) && ADLC.sr2pse) {	// PSE active?
			ADLC.sr2pse++;						// Advance PSE to next priority
			if (ADLC.sr2pse > 4)
				ADLC.sr2pse = 0;
		} else {
			ADLC.sr2pse = 0;
		}

		sr1b2cause = 0;							// clear cause of sr2b1 going up
		if (ADLC.control1 & 64) {				// rx reset,clear buffers.
			EconetRx.Pointer =0;
			EconetRx.BytesInBuffer = 0;
			ADLC.rxfptr = 0;
			ADLC.rxap = 0;
			ADLC.rxffc = 0;
			ADLC.sr2pse = 0;
		}
//        fourwaystage = FWS_IDLE;              // this really doesn't like being here.
	}

	// CR2b6 - CLT TxST - Clear Transmitter Status - reset status bits
	if ((ADLC.control2 & 64) || (ADLC.control1 & 128)) {	// or txreset
		ADLC.control2 &= ~64;					// clear this bit
		ADLC.status1 &= ~0x70;					// clear TXU , cts, TDRA/FC
		if (ADLC.cts) {
			ADLC.status1 |= 16;					//cts follows signal, reset high again
			ADLCtemp.status1 |= 16;				// don't trigger another interrupt instantly
		}
		if (ADLC.control1 & 128) {				// tx reset,clear buffers.
			EconetTx.Pointer =0;
			EconetTx.BytesInBuffer = 0;
			ADLC.txfptr = 0;
			ADLC.txftl = 0;
		}
	}

	// CR2b7 - RTS control - looks after RTS output line. ignored here.
	//		but used in CTS logic
	// RTS gates TXD onto the econet bus. if not zero, no tx reaches it,
	// in the B+, RTS substitutes for the collision detection circuit.

	// -----------------------
	// CR3 seems always to be all zero while debugging emulation.
	// CR3b0 - LCF - Logical Control Field Select. if zero, no control fields in frame, ignored.
	// CR3b1 - CEX - Extend Control Field Select - when set, control field is 16 bits. ignored.
	// CR3b2 - AEX - When set, address will be two bytes (unless first byte is zero). ignored here.
	// CR3b3 - 01/11 idle - idle transmission mode - ignored here,.
	// CR3b4 - FDSE - flag detect status enable.  when set, then FD (SR1b3) + interrupr indicated a flag
	//				has been received. I don't think we use this mode, so ignoring it.
	// CR3b5 - Loop - Loop mode. Not used.
	// CR3b6 - GAP/TST - sets test loopback mode (when not in Loop operation mode.) ignored.
	// CR3b7 - LOC/DTR - (when not in loop mode) controls DTR pin directly. pin not used in a BBC B

	// -----------------------
	// CR4b0 - FF/F - when clear, re-used the Flag at end of one packet as start of next packet. ignored.
	// CR4b1,2 - TX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b3,4 - RX word length. 11=8 bits. BBC uses 8 bits so ignore flags and assume 8 bits throughout
	// CR4b5 - TransmitABT - Abort Transmission.  Once abort starts, bit is cleared.
	if (ADLC.control4 & 32) {		// ABORT
		if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, "EconetPoll: TxABORT is set\0");
		ADLC.txfptr = 0;			//	reset fifo
		ADLC.txftl = 0;				//	reset fifo flags
		EconetTx.Pointer = 0;
		EconetTx.BytesInBuffer = 0;
		ADLC.control4 &= ~32;		// reset flag.
        fourwaystage = FWS_IDLE;
        if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, "Econet: Set FWS_IDLE (abort)");
	}

	// CR4b6 - ABTex - extend abort - adjust way the abort flag is sent.  ignore,
	//	can affect timing of RTS output line (and thus CTS input) still ignored.
	// CR4b7 - NRZI/NRZ - invert data encoding on wire. ignore.


	//--------------------------------------------------------------------------------------------
	//--------------------------------------------------------------------------------------------

	if (EconetTrigger<=TotalCycles) {
		// Only do this bit occasionally as data only comes in from
		// line occasionally.
		// Trickle data between fifo registers and ip packets.

		// Transmit data
		if (!(ADLC.control1 & 128)) {		// tx reset off
			if (ADLC.txfptr) {				// there is data is in tx fifo
				if (DebugEnabled)
					DebugDisplayTrace(DEBUG_ECONET, true, "EconetPoll: Write to FIFO noticed");
				int TXlast = false;
				if (ADLC.txftl & powers[ADLC.txfptr-1]) TXlast=TRUE;	// TxLast set
				if (EconetTx.Pointer + 1 >sizeof(EconetTx.buff) || // overflow IP buffer
						(ADLC.txfptr >4 )) {				// overflowed fifo
					ADLC.status1 |= 32;						// set tx underrun flag
					EconetTx.Pointer = 0;				// wipe buffer
					EconetTx.BytesInBuffer = 0;
					ADLC.txfptr = 0;
					ADLC.txftl = 0;
					if (DebugEnabled)
						DebugDisplayTrace(DEBUG_ECONET, true, "EconetPoll: TxUnderun!!");
				} else {
					EconetTx.buff[EconetTx.Pointer] = ADLC.txfifo[--ADLC.txfptr];
					EconetTx.Pointer++;
				}
				if (TXlast) {	// TxLast set
					if (DebugEnabled) {
						sprintf(info, "Econet: TXLast set - Send packet to %02x %02x ",
								(unsigned int)(EconetTx.buff[1]), (unsigned int)EconetTx.buff[0]);
						DebugDisplayTrace(DEBUG_ECONET, true, info);
					}
					// first two bytes of Econettxbuff contain the destination address
					// (or one zero byte for broadcast)
					sockaddr_in RecvAddr;
                    bool SendMe = false;
                    int SendLen;
                    unsigned int i=0;
                    if (confAUNmode && (BeebTx.eh.deststn == 255 || BeebTx.eh.deststn == 0)) { // Broadcast
                       // TODO something
                       // Somewhere that I cannot suggest now fing suggested that
                       // sun buffers broadcast packet, and broadcasts a simple flag. stations
                       // poll us to get the actual broadcast data ..
                       // Hmmm...
                       //
                       // OK, just send it to the local broadcast address.
                       // TODO lookup destnet in aunnet() and use proper id address!ourselves
                       RecvAddr.sin_family = AF_INET;
                       RecvAddr.sin_port = htons(32768);
                       RecvAddr.sin_addr.s_addr = INADDR_BROADCAST; //((EconetListenIP & 0x00FFFFFF) | 0xFF000000) 
                       SendMe = true; 
                    } else {
                        do {
							// does the packet match this network table entry?
							// SendMe = false;
							// check for 0.stn and mynet.stn.
							// aunnet wont be populated if not in aun mode, but we don't need to not check
							// it because it won't matter..
							if ((network[i].network == (unsigned int)(BeebTx.eh.destnet)
								|| network[i].network == aunnet[myaunnet].network ) &&
								network[i].station == (unsigned int)(BeebTx.eh.deststn)) {
									SendMe = true;
									break;
							}
							i++;
                        }
                        while (i < networkp);
                        // guess address if not found in table 
                        if (!SendMe && confSTRICT) { // dodm't find ot and allowed to guess
                            if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, "Econet: Send to unknown host; make assumptions & add entry!");
                            if (BeebTx.eh.destnet == 0 || BeebTx.eh.destnet == aunnet[myaunnet].network) {
                                network[i].inet_addr = aunnet[myaunnet].inet_addr | (BeebTx.eh.deststn << 24);
                                network[i].port = 32768;  // default AUN port 
                                network[i].network = BeebTx.eh.destnet;
                                network[i].station = BeebTx.eh.deststn;
                                SendMe = true;
                                network[++networkp].station = 0;
                            } else {
                                unsigned int j = 0;
                                do {
                                    if (aunnet[j].network == BeebTx.eh.deststn) { 
                                        network[i].inet_addr = aunnet[j].inet_addr | (BeebTx.eh.deststn << 24);
                                        network[i].port = 32768;  // default AUN port
                                        network[i].network = BeebTx.eh.destnet;
                                        network[i].station = BeebTx.eh.deststn;
                                        SendMe = true;
                                        network[++networkp].station = 0;
                                        break;
                                 }
                                j++;
                            } while (j < aunnetp);
                        }
                    }                

                    RecvAddr.sin_family = AF_INET;
                    RecvAddr.sin_port = htons(network[i].port);
                    RecvAddr.sin_addr.s_addr = network[i].inet_addr;
                    }

					do {
						// Send to all stations except ourselves
						if (network[i].station != EconetStationNumber) {
							RecvAddr.sin_family = AF_INET;
							RecvAddr.sin_port = htons(network[i].port);
							RecvAddr.sin_addr.s_addr = network[i].inet_addr;

							// Send a datagram to the receiver
							if (DebugEnabled) {
								sprintf(info, "Econet: TXLast set - Send %d byte packet to %02x %02x (%08X :%u)",
										EconetTx.Pointer,
										(unsigned int)(EconetTx.buff[1]), (unsigned int)EconetTx.buff[0],
										(unsigned int)network[i].inet_addr, (unsigned int)network[i].port);
								DebugDisplayTrace(DEBUG_ECONET, true, info);
								sprintf(info, "Econet: Packet data:");
								for (unsigned int i = 0; i < EconetTx.Pointer; ++i) {
									sprintf(info+strlen(info), " %02X", EconetTx.buff[i]);
								}
								DebugDisplayTrace(DEBUG_ECONET, true, info);
							}

							if (sendto(SendSocket, (char *)EconetTx.buff, EconetTx.Pointer,
								   0, (sockaddr *) &RecvAddr, sizeof(RecvAddr)) == SOCKET_ERROR) {
								sprintf(info, "Econet: Failed to send packet to %02x %02x (%08X :%u)",
									(unsigned int)(EconetTx.buff[1]), (unsigned int)EconetTx.buff[0],
									(unsigned int)network[i].inet_addr, (unsigned int)network[i].port);
								EconetError(info);
							}
						}
						i++;
					} while (network[i].station != 0);

					// Sending packet will mean peer goes into flag fill while
					// it deals with it
					FlagFillActive = true;
					SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);
					if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, "Econet: FlagFill set");

					// When the application is finished sending, close the socket.
					//	    closesocket(SendSocket);
					EconetTx.Pointer = 0;					// wipe buffer
					EconetTx.BytesInBuffer = 0;
					if (DebugEnabled) debugADLCprint();
				}	
			}	
		}

		// Receive data
		if (!(ADLC.control1 & 64)) {		// rx reset off
			if (EconetRx.Pointer < EconetRx.BytesInBuffer) {
				// something waiting to be given to the processor
				if (ADLC.rxfptr<3 )	{		// space in fifo
					if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true,
										  "EconetPoll: Time to give another byte to the beeb");
					ADLC.rxfifo[2] = ADLC.rxfifo[1];
					ADLC.rxfifo[1] = ADLC.rxfifo[0];
					ADLC.rxfifo[0] = EconetRx.buff[EconetRx.Pointer];
					ADLC.rxfptr++;
					ADLC.rxffc = (ADLC.rxffc <<1) & 7;
					ADLC.rxap = (ADLC.rxap <<1) & 7;
					if (EconetRx.Pointer == 0)
						ADLC.rxap |= 1; 			// todo - 2 bytes? adr extention mode
					EconetRx.Pointer++;
					if (EconetRx.Pointer >= EconetRx.BytesInBuffer)  { // that was last byte!
						ADLC.rxffc |= 1;			// set FV flag (this was last byte of frame)
						EconetRx.Pointer = 0;    // Reset read for next packet
						EconetRx.BytesInBuffer = 0;
					}		
				}
			}
			if (ADLC.rxfptr == 0)  {
				// still nothing in buffers (and thus nothing in Econetrx buffer)
				ADLC.control1 &= ~32;		// reset discontinue flag

				// wait for cpu to clear FV flag from last frame received
				if (!(ADLC.status2 & 2)) {
					// Try and get another packet from network
					// Check if packet is waiting without blocking
  					int RetVal;
					fd_set RdFds;
					timeval TmOut = {0,0};
					FD_ZERO(&RdFds);
					FD_SET(ListenSocket, &RdFds);
					RetVal = select(ListenSocket + 1, &RdFds, NULL, NULL, &TmOut);
					if (RetVal > 0)
					{
						// Read the packet
						RetVal = recv(ListenSocket, (char *)EconetRx.buff, sizeof(EconetRx.buff), 0);
  						if (RetVal > 0) {
							if (DebugEnabled) {
								sprintf (info, "EconetPoll: Packet received. %u bytes", (int)RetVal);
								DebugDisplayTrace(DEBUG_ECONET, true, info);
								sprintf (info, "EconetPoll: Packet data:");
								for (int i = 0; i < RetVal; ++i) {
									sprintf(info+strlen(info), " %02X", EconetRx.buff[i]);
								}
								DebugDisplayTrace(DEBUG_ECONET, true, info);
							}
							EconetRx.Pointer =0;
							EconetRx.BytesInBuffer = RetVal;

							if (EconetRx.buff[0] == EconetStationNumber) {
								// Peer sent us packet - no longer in flag fill
								FlagFillActive = false;
								if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true,
													"Econet: FlagFill reset");
							} else {
								// Two other stations communicating - assume one of them will flag fill
								FlagFillActive = true;
								SetTrigger(EconetFlagFillTimeout, EconetFlagFillTimeoutTrigger);
								if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true,
													"Econet: FlagFill set - other station comms");
							}
						} else if (RetVal == SOCKET_ERROR) {
							EconetError("Econet: Failed to receive packet");
						}
					} else if (RetVal == SOCKET_ERROR) {
						EconetError("Econet: Failed to check for new packet");
					}
				} 
			}
		}

		// Update idle status
		if (!(ADLC.control1 & 0x40)				// not rxreset
			&& !ADLC.rxfptr						// nothing in fifo
			&& !(ADLC.status2 & 2)              // no FV
			&& (EconetRx.BytesInBuffer ==0)) {	// nothing in ip buffer
			ADLC.idle = TRUE;
		} else {
			ADLC.idle = FALSE;
		}

		//----------------------------------------------------------------------------------
		// how long before we come back in here?
	    SetTrigger(TIMEBETWEENBYTES,EconetTrigger);
	}

	// Reset pseudo flag fill?
	if (EconetFlagFillTimeoutTrigger<=TotalCycles && FlagFillActive) {
		FlagFillActive = false;
		if (DebugEnabled)
			DebugDisplayTrace(DEBUG_ECONET, true, "Econet: FlagFill timeout reset");
	}	


	//--------------------------------------------------------------------------------------------
	// Status bits need changing?

	// SR1b0 - RDA - received data available. 
	if (!(ADLC.control1 & 64)) { 		// rx reset off
		if ((ADLC.rxfptr && !(ADLC.control2 & 2))			// 1 byte mode
		    || ((ADLC.rxfptr > 1) && (ADLC.control2 & 2)))	// 2 byte mode
		{
			ADLC.status1 |=  1;		// set RDA copy
			ADLC.status2 |=  128;
		} else {
			ADLC.status1 &= ~1;
			ADLC.status2 &= ~128;
		} 
	}
	// SR1b1 - S2RQ - set after SR2, see below
	// SR1b2 - LOOP - set if in loop mode. not supported in this emulation,
	// SR1b3 - FD - Flag detected. Hmm.
	// SR1b4 - CTS - set by ~CTS line going up, and causes IRQ if enabled.
	//				only cleared by cpu. 
	//			~CTS is a NAND of DCD(clock present)(high if valid)
	//			 &  collission detection!
	//			i.e. it's low (thus clear to send) when we have both DCD(clock) 
	//          present AND no collision on line and no collision.
	//          cts will ALSO be high if there is no cable!
	//		we will only bother checking against DCD here as can't have collisions.
	//		but nfs then loops waiting for CTS high!	
	//  on the B+ there is (by default) no collission detection circuitary. instead S29
	// links RTS in it's place, thus CTS is a NAND of not RTS & not DCD
	// i.e. cts = ! ( !rts && !dcd ) all signals are active low.
	// there is a delay on rts going high after cr2b7=0 - ignore this for now.
	// cr2b7 = 1 means RTS low means not rts high means cts low
	// sockets true means dcd low means not dcd high means cts low
	// doing it this way finally works !!  great :-) :-)

	if (ReceiverSocketsOpen && (ADLC.control2 & 128) ) {	// clock + RTS
		ADLC.cts = FALSE;
		ADLC.status1 &= ~16;
	} else {
		ADLC.cts = TRUE;
	}

	// and then set the status bit if the line is high! (status bit stays
	// up until cpu tries to clear it) (& still stays up if cts line still high)

	if (!(ADLC.control1 && 128) && ADLC.cts) {
		ADLC.status1 |= 16;		// set CTS now
	}

	// SR1b5 - TXU - Tx Underrun.
	if (ADLC.txfptr>4) {		// probably not needed
		if (DebugEnabled) {
			sprintf(info, "Econet: TX Underrun - TXfptr %02x", (unsigned int)ADLC.txfptr);
			DebugDisplayTrace(DEBUG_ECONET, true, info);
		}
		ADLC.status1 |= 32;
		ADLC.txfptr = 4;
	}

	// SR1b6 TDRA flag - another complicated derivation
	if (!(ADLC.control1 & 128)) {		// not txreset
		if (!(ADLC.control2 & 8)) {		// tdra mode
			if (   (   ((ADLC.txfptr < 3) && !(ADLC.control2 & 2)) // space in fifo?
			        || ((ADLC.txfptr < 2) && (ADLC.control2 & 2))) // space in fifo?
			    && (!(ADLC.status1 & 16))		// clear to send is ok
			    && (!(ADLC.status2 & 32)) ) {	// DTR not high

				if (DebugEnabled && !(ADLC.status1 & 64))
					DebugDisplayTrace(DEBUG_ECONET, true, "set tdra");
				ADLC.status1 |= 64;				// set Tx Reg Data Available flag.
			} else {
				if (DebugEnabled && (ADLC.status1 & 64))
					DebugDisplayTrace(DEBUG_ECONET, true, "clear tdra");
				ADLC.status1 &= ~64;			// clear Tx Reg Data Available flag.
			}
		} else {		// FC mode
			if (!(ADLC.txfptr)) {			// nothing in fifo
				if (DebugEnabled && !(ADLC.status1 & 64))
					DebugDisplayTrace(DEBUG_ECONET, true, "set fc");
				ADLC.status1 |= 64;			// set Tx Reg Data Available flag.
			} else {
				if (DebugEnabled && (ADLC.status1 & 64))
					DebugDisplayTrace(DEBUG_ECONET, true, "clear fc");
				ADLC.status1 &= ~64;		// clear Tx Reg Data Available flag.
			}
		}
	}
	// SR1b7 IRQ flag - see below

	// SR2b0 - AP - Address present 
	if (!(ADLC.control1  & 64)) {				// not rxreset
		if (ADLC.rxfptr &&
			(ADLC.rxap & (powers[ADLC.rxfptr-1]))) {		// ap bits set on fifo
			ADLC.status2 |= 1;
		} else {
			ADLC.status2 &= ~1;
		}
		// SR2b1 - FV -Frame Valid - set in rx - only reset by ClearRx or RxReset
		if (ADLC.rxfptr &&
			(ADLC.rxffc & (powers[ADLC.rxfptr-1]))) {
			ADLC.status2 |= 2;
		}
		// SR2b2 - Inactive Idle Received - sets irq!
		if (ADLC.idle && !FlagFillActive) {
			ADLC.status2 |= 4;
		} else {
			ADLC.status2 &= ~4;
		}
	}
	// SR2b3 - RxAbort - Abort received - set in rx routines above
	// SR2b4 - Error during reception - set if error flaged in rx routine.
	// SR2b5 - DCD
	if (!ReceiverSocketsOpen) {			// is line down?
		ADLC.status2 |= 32;				// flag error
	} else {
		ADLC.status2 &= ~32;
	}
	// SR2b6 - OVRN -receipt overrun. probably not needed 
	if (ADLC.rxfptr>4) {
		ADLC.status2 |= 64;
		ADLC.rxfptr = 4;
	}
	// SR2b7 - RDA. As per SR1b0 - set above.


	// Handle PSE - only for SR2 Rx bits at the moment - others todo
	int sr2psetemp = ADLC.sr2pse;
	if (ADLC.control2 & 1) {
		if ((ADLC.sr2pse <= 1) && (ADLC.status2 & 0x7A)) {	// ERR, FV, DCD, OVRN, ABT
			ADLC.sr2pse = 1;
			ADLC.status2 &= ~0x85;
		} else if ((ADLC.sr2pse <= 2) && (ADLC.status2 & 0x04)) { // Idle
			ADLC.sr2pse = 2;
			ADLC.status2 &= ~0x81;
		} else if ((ADLC.sr2pse <= 3) && (ADLC.status2 & 0x01)) { // AP
			ADLC.sr2pse = 3;
			ADLC.status2 &= ~0x80;
		} else if (ADLC.status2 & 0x80) {					// RDA
			ADLC.sr2pse = 4;
			ADLC.status2 &= ~0x02;
		} else {
			ADLC.sr2pse = 0;				// No relevant bits set
		}

		// Set SR1 RDA copy
		if (ADLC.status2 & 0x80)
			ADLC.status1 |= 1;
		else
			ADLC.status1 &= ~1;

	} else {								// PSE inactive
		ADLC.sr2pse = 0;
	}
	if (DebugEnabled && sr2psetemp != ADLC.sr2pse) {
		sprintf(info, "ADLC: PSE SR2Rx priority changed to %d", ADLC.sr2pse);
		DebugDisplayTrace(DEBUG_ECONET, true, info);
	}


	// Do we need to flag an interrupt?
	if (ADLC.status1 != ADLCtemp.status1 || ADLC.status2 != ADLCtemp.status2) { // something changed
		unsigned char tempcause, temp2;

		// SR1b1 - S2RQ - Status2 request. New bit set in S2?
		tempcause = ((ADLC.status2 ^ ADLCtemp.status2) & ADLC.status2) & ~128;

		if (!(ADLC.control1 & 2))	{	// RIE not set,
			tempcause = 0;
		}

		if (tempcause) { //something got set
			ADLC.status1 |= 2;
			sr1b2cause = sr1b2cause | tempcause;
		} else if (!(ADLC.status2 & sr1b2cause)) { //cause has gone
			ADLC.status1 &= ~2;
			sr1b2cause = 0;
		}

		// New bit set in S1?
		tempcause = ((ADLC.status1 ^ ADLCtemp.status1) & ADLC.status1) & ~128;

		if (!(ADLC.control1 & 2))	{	// RIE not set,
			tempcause = tempcause & ~11;
		}
		if (!(ADLC.control1 & 4))	{	// TIE not set,
			tempcause = tempcause & ~0x70;
		}
			
		if (tempcause) { //something got set
			interruptnow = TRUE;
			irqcause = irqcause | tempcause;	// remember which bit went high to flag irq
			// SR1b7 IRQ flag
			ADLC.status1 |= 128;

			if (DebugEnabled) {
				sprintf(info, "ADLC: Status1 bit got set %02x, interrupt", (int)tempcause);
				DebugDisplayTrace(DEBUG_ECONET, true, info);
			}
		}

		// Bit cleared in S1?
		temp2 = ((ADLC.status1 ^ ADLCtemp.status1) & ADLCtemp.status1) & ~128;
		if (temp2) {		// something went off
			irqcause = irqcause & ~temp2;	// clear flags that went off
			if (irqcause == 0) {	// all flag gone off now
				// clear irq status bit when cause has gone.
				ADLC.status1 &= ~128;
			} else {
				// interrupt again because still have flags set
				if (ADLC.control2 & 1) {
					interruptnow = TRUE;
					DebugDisplayTrace(DEBUG_ECONET, true, "ADLC: S1 flags still set, interrupt");
				}
			}
			if (DebugEnabled) {
				sprintf(info, "ADLC: IRQ cause reset, irqcause %02x", (int)irqcause);
				DebugDisplayTrace(DEBUG_ECONET, true, info);
			}
		}
		if (DebugEnabled) debugADLCprint();
	}

	return (interruptnow);	// flag NMI if necessary. see also INTON flag as
							// this can cause a delayed interrupt.(beebmem.cpp)
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// display some information

void debugADLCprint(void) {
	char info[200];
	sprintf(info, "ADLC: Ctrl:%02X %02X %02X %02X St:%02X %02X TXFptr:%02x rx:%02x FF:%d IRQc:%02x SR2c:%02x PC:%04x  ",
			(int)ADLC.control1, (int)ADLC.control2, (int)ADLC.control3, (int)ADLC.control4,
			(int)ADLC.status1, (int)ADLC.status2,
			(int)ADLC.txfptr, (int)ADLC.rxfptr, FlagFillActive ? 1 : 0,
			(int)irqcause, (int)sr1b2cause, (int)ProgramCounter);
	DebugDisplayTrace(DEBUG_ECONET, true, info);
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// Display an error message box

void EconetError(const char *errstr) {
	if (DebugEnabled) DebugDisplayTrace(DEBUG_ECONET, true, errstr);
	fprintf(stderr, "%s\n", errstr);
}

