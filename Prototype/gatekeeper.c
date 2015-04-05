/**
 * Routes data between internal BoidCPUs, filters out irrelevant external data
 * and forwards internal data to external entities as appropriate.
 */

#include "stdlib.h"			// For rand()
#include "xparameters.h"	// Xilinx definitions
#include "xemaclite.h"		// Ethernet
#include "xuartlite_l.h"    // UART
#include "fsl.h"        	// AXI Steam
#include "boids.h"			// Boid definitions
#include <stdint.h>			// For specific data types e.g. int16_t

#define RESIDENT_BOIDCPU_COUNT	2	// The number of resident BoidCPUs
#define BOID_DATA_LENGTH 		3

#define BOIDMASTER_CHANNEL		0
#define BOIDCPU_CHANNEL_1		1
#define BOIDCPU_CHANNEL_2		2
#define ALL_BOIDCPU_CHANNELS	99	// When a message is sent to all channels

#define MASTER_IS_RESIDENT		1	// Defined when the BoidMaster is resident

#define KILL_KEY				0x6B// 'k'
#define PAUSE_KEY				0x70// 'p'

#define EXTERNAL_RECIPIENT				0
#define INTERNAL_RECIPIENT				1
#define INTERNAL_AND_EXTERNAL_RECIPIENT	2

// Setup Ethernet and its transmit and receive buffers
// FIXME: Ethernet requires a u8 buffer, but FXL requires a u32...
XEmacLite ether;
u8 externalOutput[MAX_CMD_LEN * MAX_OUTPUT_CMDS];
u8 externalInput[MAX_CMD_LEN * MAX_INPUT_CMDS];

// Setup other variables
#ifdef MASTER_IS_RESIDENT
u8 channelSetupCounter = 1;
u8 discoveredBoidCPUCount = 0;
u8 channelIDList[RESIDENT_BOIDCPU_COUNT + 1];
#else
u8 channelSetupCounter = 0;
u8 channelIDList[RESIDENT_BOIDCPU_COUNT];
#endif

// If the BoidMaster is present, then it must be on channel 0

u32 messageData[MAX_CMD_BODY_LEN];
u32 gatekeeperID = 0;
bool boidCPUsSetup = false;
bool fowardMessage = true;
bool forwardingInterceptedSetup = false;
u8 ackCount = 0;

u8 residentNbrCounter = 0;
u8 residentBoidCPUNeighbours[MAX_BOIDCPU_NEIGHBOURS * RESIDENT_BOIDCPU_COUNT];

// Protocols
void checkForInput();

void respondToPing();
void interceptSetupInfo(u32 *interceptedData);

u8 internalChannelLookUp(u32 to);
u8 recipientLookUp(u32 to);
bool externalMessageRelevant();
void interceptMessage(u32 *data);

void processReceivedExternalMessage();
void processReceivedInternalMessage(u32 *inputData);

void sendMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);

void printMessage(bool send, u32 *data);
void decodeAndPrintBoids(u32 *data);

#ifdef MASTER_IS_RESIDENT
void takeUserInput();
void uiBoidCPUSearch();
void sendKillCommand();
#endif

void putFSLData(u32 value, u32 channel);
u32 getFSLData(u32 *data, u32 channel);

//============================================================================//
//- Main Method --------------------------------------------------------------//
//============================================================================//

// TODO: Rename to main() when deployed
int main() {
	// Setup the Ethernet
	XEmacLite_Config *etherconfig = XEmacLite_LookupConfig(
			XPAR_EMACLITE_0_DEVICE_ID);
	XEmacLite_CfgInitialize(&ether, etherconfig, etherconfig->BaseAddress);

	// Generate random, temporary Gatekeeper ID
	// TODO: Ensure that this doesn't clash with the BoidIDs
	gatekeeperID = rand();

	do {
		bool simulationKilled = false;
		do {
#ifdef MASTER_IS_RESIDENT
			// Start the user interface
			print("------------------------------------------------------\n\r");
			print("---------FPGA Implementation of Flocking Birds--------\n\r");
			print("------------------------------------------------------\n\r");

			// Search for BoidCPUs
			uiBoidCPUSearch();

			// Take user input e.g. simulation boid count
			takeUserInput();
#endif
			// Finally, begin main loop
			do {
				checkForInput();

#ifdef MASTER_IS_RESIDENT
				// Handle a kill key or a pause key being pressed
				if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
					int key = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);

					if (key == KILL_KEY) {
						simulationKilled = true;
						sendKillCommand();
						print("Simulation killed, restarting...\n\r");
					} else if (key == PAUSE_KEY) {
						print("Simulation paused, press 'p' to resume\n\r");
						bool simulationUnpaused = false;
						do {
							if (XUartLite_RecvByte(
									XPAR_RS232_UART_1_BASEADDR) == PAUSE_KEY) {
								simulationUnpaused = true;
							}
						} while (!simulationUnpaused);
						print("Simulation resumed\n\r");
					} else if (key == 0x67) {
						print("Issuing BoidGPU ACK...\n\r");
						sendInternalMessage(0, CONTROLLER_ID, BOIDGPU_ID,
								CMD_ACK, messageData);
						print("----------------------------------------------"
							"------------------------------------------------"
							"----------------\n\r");
						print("\n\r------------------------------------------"
							"------------------------------------------------"
							"--------------------\n\r");
					}
				}
#endif
			} while (!simulationKilled);
		} while (!simulationKilled);
	} while (1);

	return 0;
}

//============================================================================//
//- Message Transfer ---------------------------------------------------------//
//============================================================================//

/**
 * The main method used to determine if there is any input available on either
 * internal or external lines. If there is, it is processed accordingly.
 */
void checkForInput() {
	// Check for external (Ethernet) data --------------------------------------
	volatile int recv_len = 0;
	recv_len = XEmacLite_Recv(&ether, externalInput);

	// Process received external data ------------------------------------------
	if (recv_len != 0) {
		processReceivedExternalMessage();
	}

	// Check for internal (FXL/AXI) data ---------------------------------------
	u32 channelZeroData[MAX_CMD_LEN];
	int channelZeroInvalid = getFSLData(channelZeroData, BOIDMASTER_CHANNEL);

	u32 channelOneData[MAX_CMD_LEN];
	int channelOneInvalid = getFSLData(channelOneData, BOIDCPU_CHANNEL_1);

	u32 channelTwoData[MAX_CMD_LEN];
	int channelTwoInvalid = getFSLData(channelTwoData, BOIDCPU_CHANNEL_2);

	// Process received internal data ------------------------------------------
	if (!channelZeroInvalid) {
		processReceivedInternalMessage(channelZeroData);
	}

	if (!channelOneInvalid) {
		processReceivedInternalMessage(channelOneData);
	}

	if (!channelTwoInvalid) {
		processReceivedInternalMessage(channelTwoData);
	}
}

//============================================================================//
//- Message Reception Functions ----------------------------------------------//
//============================================================================//

void processReceivedInternalMessage(u32 *inputData) {
	fowardMessage = true;
	print("INTERNAL: ");
	printMessage(false, inputData);

	if ((!boidCPUsSetup) && (inputData[CMD_TYPE] != CMD_ACK)) {
		interceptMessage(inputData);
	}

	// Collect the ACKs for the recipient BoidCPUs and issue a collective one
	if (inputData[CMD_TYPE] == CMD_ACK) {
		fowardMessage = false;
		ackCount++;

		if (ackCount == RESIDENT_BOIDCPU_COUNT) {
			print ("All ACKs received \n\r");
			sendMessage(0, CONTROLLER_ID, gatekeeperID, CMD_ACK, messageData);
			ackCount = 0;
			print("-----------------------------------------------------------"
					"---------------------------------------------------\n\r");
		} else {
			xil_printf("Waiting for ACKs (received %d of %d)...\n\r", ackCount,
					RESIDENT_BOIDCPU_COUNT);
		}
	}

	// Forward the message, if needed
	if (fowardMessage) {
		u32 inputDataBodyLength = inputData[CMD_LEN] - CMD_HEADER_LEN;
		u32 inputDataBody[inputDataBodyLength];
		int i = 0;
		for (i = 0; i < inputDataBodyLength; i++) {
			inputDataBody[i] = inputData[CMD_HEADER_LEN + i];
		}

		sendMessage(inputDataBodyLength, inputData[CMD_TO],
				inputData[CMD_FROM], inputData[CMD_TYPE], inputDataBody);
	}
}

void processReceivedExternalMessage() {
	print("EXTERNAL: ");
	printMessage(false, (u32*)externalInput);

	if (!boidCPUsSetup) {
		interceptMessage((u32*) externalInput);
	} else if (externalMessageRelevant()) {
		// Forward the message
		u32 dataBodyLength = externalInput[CMD_LEN] - CMD_HEADER_LEN;
		u32 dataBody[dataBodyLength];
		int i = 0;
		for (i = 0; i < dataBodyLength; i++) {
			dataBody[i] = externalInput[CMD_HEADER_LEN + i];
		}

		sendMessage(dataBodyLength, externalInput[CMD_TO],
				externalInput[CMD_FROM], externalInput[CMD_TYPE], dataBody);
	}
}

//============================================================================//
//- Message Transmission Functions -------------------------------------------//
//============================================================================//

void sendMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
	u8 recipientLocation;

	// If the BoidCPUs are not yet set up, a setup message is being sent and
	// the message is addressed to this Gatekeeper, send internally. If it is
	// not addressed to this Gatekeeper, send it externally.
	if ((!boidCPUsSetup) && (type == CMD_SIM_SETUP)) {
		print("BoidCPUs not setup and setup message being sent...\n\r");

		if ((to == gatekeeperID) || (forwardingInterceptedSetup)) {
			recipientLocation = INTERNAL_RECIPIENT;
		} else {
			recipientLocation = EXTERNAL_RECIPIENT;
		}
	}
#ifdef MASTER_IS_RESIDENT
	// Don't forward a ping to internal BoidCPUs
	else if (type == CMD_PING) {
		recipientLocation = EXTERNAL_RECIPIENT;
	}
#endif
	else {
		recipientLocation = recipientLookUp(to);
	}

	// If MASTER_IS_PRESENT and CMD_PING
	// 	Then recipient is EXTERNAL

	// Send a message internally or externally, depending on the recipient
	if (recipientLocation == EXTERNAL_RECIPIENT) {
		sendExternalMessage(len, to, from, type, data);
	} else if (recipientLocation == INTERNAL_RECIPIENT) {
		sendInternalMessage(len, to, from, type, data);
	} else if (recipientLocation == INTERNAL_AND_EXTERNAL_RECIPIENT) {
		sendExternalMessage(len, to, from, type, data);
		sendInternalMessage(len, to, from, type, data);
	}
}

/**
 * Sends a message internally, over the FSL/AXI bus/stream, to BoidCPUs that
 * reside on the same FPGA that the MicroBlaze does.
 */
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
	u8 channel;

	// If the BoidCPUs are not yet setup and a setup message is being sent,
	// send the message down a specific channel
	if ((!boidCPUsSetup) && (type == CMD_SIM_SETUP)) {
		channel = channelSetupCounter;
	} else {
		channel = internalChannelLookUp(to);
	}

	// First, create the message
	u32 command[MAX_CMD_LEN];
	int i = 0, j = 0;

	command[CMD_LEN] = len + CMD_HEADER_LEN;
	command[CMD_TO] = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		for (i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}

	// Multicast messages - don't send back to self
	// For each channel, if the multicast message is not from that channel then
	// send the message down that channel.
	if (to == CMD_MULTICAST) {
#ifdef MASTER_IS_RESIDENT
		for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
		for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
			if (channelIDList[i] != from) {
//				xil_printf("INTERNAL - Sending to %d (channel %d)...\n\r", channelIDList[i], i);
				print("INTERNAL: ");
				printMessage(true, command);
				for (j = 0; j < CMD_HEADER_LEN + len; j++) {
					putFSLData(command[j], i);
				}
			}
		}
	} else {
		// Print out message details
		if (channel == ALL_BOIDCPU_CHANNELS) {
//			print("INTERNAL - Sending to all BoidCPU channels...\n\r");
		} else {
//			xil_printf("INTERNAL - Sending to channel %d...\n\r", channel);
		}
		print("INTERNAL: ");
		printMessage(true, command);

		// Finally, send the message
		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
			switch (channel) {
			case BOIDMASTER_CHANNEL:
				putFSLData(command[i], BOIDMASTER_CHANNEL);
				break;
			case BOIDCPU_CHANNEL_1:
				putFSLData(command[i], BOIDCPU_CHANNEL_1);
				break;
			case BOIDCPU_CHANNEL_2:
				putFSLData(command[i], BOIDCPU_CHANNEL_2);
				break;
			default:
				// Otherwise, send to all BoidCPU channels
	#ifdef MASTER_IS_RESIDENT
				putFSLData(command[i], BOIDCPU_CHANNEL_1);
				putFSLData(command[i], BOIDCPU_CHANNEL_2);
	#else
				putFSLData(command[i], 0);
				putFSLData(command[i], 1);
	#endif
				break;
			}
		}


	//	switch (channel) {
	//	case BOIDMASTER_CHANNEL:
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], BOIDMASTER_CHANNEL);
	//		}
	//		break;
	//	case BOIDCPU_CHANNEL_1:
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], BOIDCPU_CHANNEL_1);
	//		}
	//		break;
	//	case BOIDCPU_CHANNEL_2:
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], BOIDCPU_CHANNEL_2);
	//		}
	//		break;
	//	default:
	//		// Otherwise, send to all BoidCPU channels
	//#ifdef MASTER_IS_RESIDENT
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], BOIDCPU_CHANNEL_1);
	//		}
	//
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], BOIDCPU_CHANNEL_2);
	//		}
	//#else
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], 0);
	//		}
	//
	//		for (i = 0; i < CMD_HEADER_LEN + len; i++) {
	//			putFSLData(command[i], 1);
	//		}
	//#endif
	//		break;
	//	}
	}
}

/**
 * Sends a message to an external entity beyond the FPGA that the MicroBlaze
 * resides on. Ethernet is used to transmit the messages to other FPGAs.
 */
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
	// First, create the message
	u32 command[MAX_CMD_LEN];
	int i = 0;

	command[CMD_LEN] = len + CMD_HEADER_LEN;
	command[CMD_TO] = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		for (i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}

	print("EXTERNAL: ");
	printMessage(true, command);

	// Then, populate the transmit buffer
	for (i = 0; i < command[CMD_LEN]; i++) {
		externalOutput[i] = command[i];
	}

	// Finally, clear the receive buffer before sending
	XEmacLite_FlushReceive(&ether);
	XEmacLite_Send(&ether, externalOutput, command[CMD_LEN]);
}

//============================================================================//
//- Message Transceive Supporting Functions ----------------------------------//
//============================================================================//

/**
 * Determine if a received external message should be forwarded internally or
 * ignored. It should be forwarded internally if:
 *  - it is from the BoidMaster
 *  - it is a broadcast command
 *  - it is from a BoidCPU that is a neighbour of a resident BoidCPU
 *  - it is addressed to the BoidMaster AND the BoidMaster is resident
 */
bool externalMessageRelevant() {
	bool result = false;

	if (externalInput[CMD_FROM] == CONTROLLER_ID) {
		result = true;
	} else if (externalInput[CMD_TO] == CMD_BROADCAST) {
		result = true;
	} else if (externalInput[CMD_FROM] >= FIRST_BOIDCPU_ID) {
		int i = 0;
		for (i = 0; i < residentNbrCounter; i++) {
			if (externalInput[CMD_FROM] == residentBoidCPUNeighbours[i]) {
				result = true;
			}
		}
	}

#ifdef MASTER_IS_RESIDENT
	else if (externalInput[CMD_TO] == CONTROLLER_ID) {
		result = true;
	}
#endif

	return result;
}

/**
 * Determine whether the message recipient is internal or external to the
 * Gatekeeper, or whether the message needs to be send both internally and
 * externally.
 */
u8 recipientLookUp(u32 to) {
	u8 recipientInterface;

	switch (to) {
		case CMD_BROADCAST:
			recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
			break;
		case CONTROLLER_ID:
#ifdef MASTER_IS_RESIDENT
			recipientInterface = INTERNAL_RECIPIENT;
#else
			recipientInterface = EXTERNAL_RECIPIENT;
#endif
			break;
		case BOIDGPU_ID:
			recipientInterface = EXTERNAL_RECIPIENT;
			break;
		case CMD_MULTICAST:
			// TODO: Determine whether all the neighbours are internal or not
			recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
			break;
		default:
			if (to >= FIRST_BOIDCPU_ID) {
				bool internal = false;
				int i = 0;
#ifdef MASTER_IS_RESIDENT
				for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
				for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
					if (channelIDList[i] == to) {
						recipientInterface = INTERNAL_RECIPIENT;
						internal = true;
						break;
					}
				}
				if(!internal) recipientInterface = EXTERNAL_RECIPIENT;

			} else {
				recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
			}

			break;
	}

	return recipientInterface;
}

//============================================================================//
//- Message Interception Functions -------------------------------------------//
//============================================================================//

/**
 * If the BoidCPUs are not yet setup, intercept certain messages. A ping
 * message is intercepted and the Gatekeeper responds on behalf of the
 * BoidCPUs. The BoidCPU setup information is intercepted in order for the
 * Gatekeeper to know the IDs of its resident BoidCPUs. A ping reply is
 * intercepted, if the BoidMaster is present, and outputted to the UI.
 */
void interceptMessage(u32 *interceptedData) {
	if (interceptedData[CMD_TYPE] == CMD_PING) {
		respondToPing();
	} else if ((interceptedData[CMD_TO] == gatekeeperID)
			&& (interceptedData[CMD_TYPE] == CMD_SIM_SETUP)) {
		fowardMessage = false;
		forwardingInterceptedSetup = true;
		interceptSetupInfo(interceptedData);
		forwardingInterceptedSetup = false;
	}

#ifdef MASTER_IS_RESIDENT
	else if (interceptedData[CMD_TYPE] == CMD_PING_REPLY) {
		xil_printf("found %d..", interceptedData[CMD_HEADER_LEN]);
		discoveredBoidCPUCount += interceptedData[CMD_HEADER_LEN];

		// Forward the data
		int i = 0;
		u32 dataBodyLength = interceptedData[CMD_LEN] - CMD_HEADER_LEN;
		u32 dataBody[dataBodyLength];
		for (i = 0; i < dataBodyLength; i++) {
			dataBody[i] = interceptedData[CMD_HEADER_LEN + i];
		}

		sendMessage(dataBodyLength, CMD_BROADCAST, interceptedData[CMD_FROM],
				interceptedData[CMD_TYPE], dataBody);
	}
#endif
}

/**
 * When the Gatekeeper detects a ping message, it responds on behalf of the
 * resident BoidCPUs with the number of resident BoidCPUs.
 */
void respondToPing() {
	print("Gatekeeper generating ping response...\n\r");

	messageData[0] = RESIDENT_BOIDCPU_COUNT;
	sendMessage(1, CONTROLLER_ID, gatekeeperID, CMD_PING_REPLY, messageData);

#ifdef MASTER_IS_RESIDENT
	xil_printf("found %d..", RESIDENT_BOIDCPU_COUNT);
	discoveredBoidCPUCount += RESIDENT_BOIDCPU_COUNT;
#endif
}

/**
 * When the Gatekeeper detects a setup command addressed to it, it extracts
 * the newly-assigned BoidCPU IDs and BoidCPU neighbours before forwarding the
 * message to one of the resident BoidCPUs.
 */
void interceptSetupInfo(u32 * setupData) {
	print("Gatekeeper intercepted setup data...\n\r");

	// Store the resident BoidCPU IDs and associated channel
	channelIDList[channelSetupCounter] = setupData[CMD_HEADER_LEN
			+ CMD_SETUP_NEWID_IDX];

	// Update Gatekeeper's neighbour list
	int i = 0, j = 0;
	for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		u8 nbr = setupData[CMD_HEADER_LEN + CMD_SETUP_BNBRS_IDX + i];
		bool neighbourAlreadyListed = false;

		for (j = 0; j < residentNbrCounter; j++) {
			if (nbr == residentBoidCPUNeighbours[j]) {
				neighbourAlreadyListed = true;
				break;
			}
		}

		if (!neighbourAlreadyListed) {
			residentBoidCPUNeighbours[residentNbrCounter] = nbr;
			residentNbrCounter++;
		}
	}

	// Forward the data
	u32 setupDataBodyLength = setupData[CMD_LEN] - CMD_HEADER_LEN;
	u32 setupDataBody[setupDataBodyLength];
	for (i = 0; i < setupDataBodyLength; i++) {
		setupDataBody[i] = setupData[CMD_HEADER_LEN + i];
	}

	sendMessage(setupDataBodyLength, CMD_BROADCAST, setupData[CMD_FROM],
			setupData[CMD_TYPE], setupDataBody);

	// Update counters
	channelSetupCounter++;
#ifdef MASTER_IS_RESIDENT
	if (channelSetupCounter == (RESIDENT_BOIDCPU_COUNT + 1)) {
#else
		if (channelSetupCounter == RESIDENT_BOIDCPU_COUNT) {
#endif
		boidCPUsSetup = true;
		print("BoidCPUs now set up\n\r");
	}
}

//============================================================================//
//- BoidMaster Support -------------------------------------------------------//
//============================================================================//

/**
 * Sends CMD_KILL to all entities in the system, on behalf of the BoidMaster.
 * Called when the user pressed the KILL_KEY.
 *
 * TODO: Need to reset a load of variables...
 */
void sendKillCommand() {
	sendExternalMessage(0, CMD_BROADCAST, gatekeeperID, CMD_KILL, messageData);
	sendInternalMessage(0, CMD_BROADCAST, gatekeeperID, CMD_KILL, messageData);
}

#ifdef MASTER_IS_RESIDENT

/**
 * Handles the process of taking user input for the simulation setup values
 */
void takeUserInput() {
	bool boidCountValid = false;	// True if count is valid
	u32 boidCount = 0;				// Number of simulation boids
	u8 index = 0;           		// Index for keyPresses
	char keyPress;          		// The key pressed
	char keyPresses[] = ""; 		// An array of the keys pressed

	while (!boidCountValid) {
		print("Enter boid count: ");

		do {
			keyPress = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);

			if (!USING_VLAB) {
				XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);
			}

			keyPresses[index] = keyPress;
			index++;
		} while (keyPress != ENTER);

		// Check if entered boid count is valid
		boidCount = (u32) atoi(keyPresses);

		if (boidCount > 0) {
			boidCountValid = true;
		} else {
			print("\n\r**Error: boid count must be greater than 0."
					" Please try again.\n\r");
		}
	}

	// If the user-entered data is valid, sent it to the controller
	messageData[0] = boidCount;
	sendInternalMessage(1, CONTROLLER_ID, gatekeeperID, CMD_USER_INFO,
			messageData);
}

/**
 * Handles the process of searching for BoidCPUs and displaying it on the UI
 */
void uiBoidCPUSearch() {
	bool boidCPUSearchComplete = false;
	do {
		print(" Searching for BoidCPUs (press ENTER to stop)...\n\r");
		sendInternalMessage(0, CONTROLLER_ID, gatekeeperID, CMD_PING_START,
				messageData);

		bool enterKeyPressed = false;
		do {
			checkForInput();

			// If the ENTER key is pressed, exit search
			if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
				if (XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR) == ENTER) {
					enterKeyPressed = true;
				}
			}
		} while (!enterKeyPressed);

		// If BoidCPUs have been found, exit search, else restart
		if (discoveredBoidCPUCount > 0) {
			boidCPUSearchComplete = true;
		} else {
			print("\n\rNo BoidCPUs found, trying again...\n\r");
		}
	} while (!boidCPUSearchComplete);

	// When the ping search is complete, inform the BoidMaster
	sendInternalMessage(0, CONTROLLER_ID, gatekeeperID, CMD_PING_END,
			messageData);
}

#endif

//============================================================================//
//- Supporting Functions -----------------------------------------------------//
//============================================================================//

/**
 * Determines the channel that at message should be sent down to reach the
 * recipient BoidCPU. If a suitable channel cannot be found, it is sent to all.
 */
u8 internalChannelLookUp(u32 to) {
	int i = 0;

#ifdef MASTER_IS_RESIDENT
	if (to == CONTROLLER_ID) {
		return BOIDMASTER_CHANNEL;
	}

	for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
		for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
		if (channelIDList[i] == to) {
			return i;
		}
	}

	return ALL_BOIDCPU_CHANNELS;	// e.g. on broadcast
}

/**
 * A wrapper for the getfsxl() method. Checks if there is any data on the
 * specified channel and reads it all in if there is. Notifies of errors.
 */
u32 getFSLData(u32 *data, u32 channel) {
	int invalid, error, value = 0;

	if (channel == BOIDCPU_CHANNEL_1)
		getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
	else if (channel == BOIDCPU_CHANNEL_2)
		getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
	else if (channel == BOIDMASTER_CHANNEL)
		getfslx(value, BOIDMASTER_CHANNEL, FSL_NONBLOCKING);

	fsl_isinvalid(invalid);          	// Was there any data?
	fsl_iserror(error);					// Was there an error?

	if (error) {
		xil_printf("Error receiving data on Channel %d: %d\n\r", channel,
				error);
	}

	if (!invalid) {
		data[CMD_LEN] = value;
//		xil_printf("Received data (Channel %d)\n\r", channel);
		int i = 0;

		// Handle invalid length values
		if ((data[CMD_LEN] == 0) || (data[CMD_LEN] > MAX_CMD_LEN)) {
			data[CMD_LEN] = MAX_CMD_LEN;
			print("Message has invalid length - correcting\n\r");
		}

		for (i = 0; i < data[CMD_LEN] - 1; i++) {
			if (channel == BOIDCPU_CHANNEL_1)
				getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
			else if (channel == BOIDCPU_CHANNEL_2)
				getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
			else if (channel == BOIDMASTER_CHANNEL)
				getfslx(value, BOIDMASTER_CHANNEL, FSL_NONBLOCKING);

			fsl_iserror(error);				// Was there an error?
			if (error) {
				xil_printf("Error receiving data on Channel %d: %d\n\r",
						channel, error);
			}

			data[i + 1] = value;
		}
	}

	return invalid;
}

/**
 * A wrapper for putting data onto the (internal) FSL bus. Can catch write
 * errors and alert when the buffer is full.
 */
void putFSLData(u32 value, u32 channel) {
	int error = 0, invalid = 0;

	// TODO: Ensure that writes use FSL_DEFAULT (blocking write)
	if (channel == BOIDCPU_CHANNEL_1)
		putfslx(value, BOIDCPU_CHANNEL_1, FSL_DEFAULT);
	else if (channel == BOIDCPU_CHANNEL_2)
		putfslx(value, BOIDCPU_CHANNEL_2, FSL_DEFAULT);
	else if (channel == BOIDMASTER_CHANNEL)
		putfslx(value, BOIDMASTER_CHANNEL, FSL_DEFAULT);

	fsl_isinvalid(invalid);
	fsl_iserror(error);

	if (invalid) {
		xil_printf("Warning - channel %d is full: %d\n\r", channel, value);
	}

	if (error) {
		xil_printf("Error writing data to channel %d: %d\n\r", channel, value);
	}
}

void printMessage(bool send, u32 *data) {
	bool drawnAlready = false;

	if (send) {
		if (data[CMD_TO] == CONTROLLER_ID) {
			print("-> TX, Gatekeeper sent command to BoidMaster:       ");
		} else if (data[CMD_TO] == CMD_BROADCAST) {
			print("-> TX, Gatekeeper sent broadcast:                   ");
		} else if (data[CMD_TO] == BOIDGPU_ID) {
			print("-> TX, Gatekeeper sent command to BoidGPU:          ");
		} else if (data[CMD_TO] == CMD_MULTICAST) {
			print("-> TX, Gatekeeper sent command to MULTICAST:        ");
		} else {
			xil_printf("-> TX, Gatekeeper sent command to %d:                ",
					data[CMD_TO]);
		}
	} else {
		if (data[CMD_FROM] == CONTROLLER_ID) {
			print("<- RX, Gatekeeper received command from BoidMaster: ");
		} else if (data[CMD_TO] == CMD_BROADCAST) {
			// This should never happen - BoidCPUs should not be able to broadcast
			xil_printf("<- RX, Gatekeeper received broadcast from %d:       ",
					data[CMD_FROM]);
		} else if (data[CMD_FROM] == BOIDGPU_ID) {
			// This should never happen
			print("<- RX, Gatekeeper received command from BoidGPU:    ");
		} else {
			xil_printf("<- RX, Gatekeeper received command from %d:          ",
					data[CMD_FROM]);
		}
	}

	switch (data[CMD_TYPE]) {
	case 0:
		print("do something                      ");
		break;
	case MODE_INIT:
		print("initialise self                   ");
		break;
	case CMD_PING:
		print("BoidCPU ping                      ");
		break;
	case CMD_PING_REPLY:
		print("BoidCPU ping response             ");
		break;
	case CMD_USER_INFO:
		print("output user info                  ");
		break;
	case CMD_SIM_SETUP:
		print("setup BoidCPU                     ");
		break;
	case MODE_CALC_NBRS:
		print("calculate neighbours              ");
		break;
	case CMD_NBR_REPLY:
		print("neighbouring boids from neighbour ");
		decodeAndPrintBoids(data);
		drawnAlready = true;
		break;
	case MODE_POS_BOIDS:
		print("calculate new boid positions      ");
		break;
	case CMD_LOAD_BAL:
		print("load balance                      ");
		break;
	case MODE_TRAN_BOIDS:
		print("transfer boids                    ");
		break;
	case CMD_BOID:
		print("boid in transit                   ");
		break;
	case MODE_DRAW:
		print("send boids to BoidGPU             ");
		break;
	case CMD_DRAW_INFO:
		print("boid info heading to BoidGPU      ");
		decodeAndPrintBoids(data);
		drawnAlready = true;
		break;
	case CMD_ACK:
		print("ACK signal                        ");
		break;
	case CMD_PING_END:
		print("end of ping                       ");
		break;
	case CMD_PING_START:
		print("start of ping                     ");
		break;
	case CMD_KILL:
		print("kill simulation                   ");
		break;
	case CMD_DEBUG:
		print("debug information                 ");
		break;
	default:
		print("UNKNOWN COMMAND                   ");
		break;
	}

	if (!drawnAlready) {
		int i = 0;
		for (i = 0; i < CMD_HEADER_LEN; i++) {
			xil_printf("%d ", data[i]);
		}
		print("|| ");

		for (i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
			xil_printf("%d ", data[CMD_HEADER_LEN + i]);
		}
		print("\n\r");
	}
}

/**
 * Decodes messages that contain encoded boids and prints out the data
 */
void decodeAndPrintBoids(u32 *data) {
	print("\n\r");
	int count = (data[CMD_LEN] - CMD_HEADER_LEN - 1) / BOID_DATA_LENGTH;

	int i = 0;
	for (i = 0; i < count; i++) {
		u32 position = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 1];
		u32 velocity = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 2];
		u32 boidID = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 3];

		int16_t xPos = ((int32_t)((int32_t)position >> 16)) >> 4;
		int16_t yPos = ((int32_t)((int16_t)position)) >> 4;

		int16_t xVel = ((int32_t)((int32_t)velocity >> 16)) >> 4;
		int16_t yVel = ((int32_t)((int16_t)velocity)) >> 4;

		xil_printf("#%d: %d %d, %d %d | ", boidID, xPos, yPos, xVel, yVel);
	}
	print("\n\r");
}

