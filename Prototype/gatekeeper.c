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

#define RESIDENT_BOIDCPU_COUNT	2	// The number of resident BoidCPUs
#define ALL_BOIDCPU_CHANNELS	99	// When a message is sent to all channels

#define MASTER_IS_RESIDENT		1	// Defined when the BoidMaster is resident

#define KILL_KEY				0x6B// 'k'
#define PAUSE_KEY				0x70// 'p'

#define EXTERNAL_RECIPIENT				0
#define INTERNAL_RECIPIENT				1
#define INTERNAL_AND_EXTERNAL_RECIPIENT	2

// Setup Ethernet and its transmit and receive buffers
// TODO: Determine maximum buffer size
// TODO: Ethernet requires a u8 buffer, but FXL requires a u32...
XEmacLite ether;
u8 externalOutput[XEL_MAX_FRAME_SIZE];
u8 externalInput[XEL_MAX_FRAME_SIZE];

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
u8 ackCount = 0;

u8 residentNbrCounter = 0;
u8 residentBoidCPUNeighbours[MAX_BOIDCPU_NEIGHBOURS * RESIDENT_BOIDCPU_COUNT];

// Protocols
void checkForInput();
void sendKillCommand();

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

#ifdef MASTER_IS_RESIDENT
void takeUserInput();
void uiBoidCPUSearch();
#endif

void putFSLData(u32 value, u32 channel);
u32 getFSLData(u32 *data, u32 channel);

//============================================================================//
//- Main Method --------------------------------------------------------------//
//============================================================================//

// TODO: Rename to main() when deployed
int mainThree() {
	// Setup the Ethernet
	XEmacLite_Config *etherconfig = XEmacLite_LookupConfig(
			XPAR_EMACLITE_0_DEVICE_ID);
	XEmacLite_CfgInitialize(&ether, etherconfig, etherconfig->BaseAddress);

	// Generate random, temporary Gatekeeper ID
//	gatekeeperID = rand();

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

				// Handle a kill key or a pause key being pressed
				if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
					if (XUartLite_RecvByte(
							XPAR_RS232_UART_1_BASEADDR) == KILL_KEY) {
						simulationKilled = true;
						sendKillCommand();
						print("Simulation killed, restarting...\n\r");
					} else if (XUartLite_RecvByte(
							XPAR_RS232_UART_1_BASEADDR) == PAUSE_KEY) {
						print("Simulation paused, press 'P' to resume\n\r");
						bool simulationUnpaused = false;
						do {
							if (XUartLite_RecvByte(
									XPAR_RS232_UART_1_BASEADDR) == PAUSE_KEY) {
								simulationUnpaused = true;
							}
						} while (!simulationUnpaused);
					}
				}
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
	int channelZeroInvalid = getFSLData(channelZeroData, 0);

	u32 channelOneData[MAX_CMD_LEN];
	int channelOneInvalid = getFSLData(channelOneData, 1);

	// Process received internal data ------------------------------------------
	if (!channelZeroInvalid) {
		processReceivedInternalMessage(channelZeroData);
	}

	if (!channelOneInvalid) {
		processReceivedInternalMessage(channelOneData);
	}
}

//============================================================================//
//- Message Reception Functions ----------------------------------------------//
//============================================================================//

void processReceivedInternalMessage(u32 *inputData) {
	fowardMessage = true;

	if (!boidCPUsSetup) {
		interceptMessage(inputData);
	}

	// Collect the ACKs for the recipient BoidCPUs and issue a collective one
	if (inputData[CMD_TYPE] == CMD_ACK) {
		fowardMessage = false;
		ackCount++;

#ifdef MASTER_IS_RESIDENT
		if (ackCount == (RESIDENT_BOIDCPU_COUNT + 1)) {
#else
		if (ackCount == RESIDENT_BOIDCPU_COUNT) {
#endif
			sendMessage(0, CONTROLLER_ID, gatekeeperID, CMD_ACK, messageData);
			ackCount = 0;
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
		if (to == gatekeeperID) {
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

	// Finally, send the message
	for (i = 0; i < CMD_HEADER_LEN + len; i++) {
		switch (channel) {
		case 0:
			putFSLData(command[i], 0);
			break;
		case 1:
			putFSLData(command[i], 1);
			break;
		default:
			// Otherwise, send to all BoidCPU channels
#ifdef MASTER_IS_RESIDENT
			putFSLData(command[i], 1);
#else
			putFSLData(command[i], 0);
			putFSLData(command[i], 1);
#endif
			break;
		}
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

	if (to == CONTROLLER_ID) {
#ifdef MASTER_IS_RESIDENT
		recipientInterface = INTERNAL_RECIPIENT;
#else
		recipientInterface = EXTERNAL_RECIPIENT;
#endif
	} else if (to == CMD_BROADCAST) {
		recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
	} else {
		recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
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
		interceptSetupInfo(interceptedData);
		fowardMessage = false;
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
	messageData[0] = RESIDENT_BOIDCPU_COUNT;
	sendMessage(1, CONTROLLER_ID, gatekeeperID, CMD_PING_REPLY, messageData);

	xil_printf("found %d..", RESIDENT_BOIDCPU_COUNT);
	discoveredBoidCPUCount += RESIDENT_BOIDCPU_COUNT;
}

/**
 * When the Gatekeeper detects a setup command addressed to it, it extracts
 * the newly-assigned BoidCPU IDs and BoidCPU neighbours before forwarding the
 * message to one of the resident BoidCPUs.
 */
void interceptSetupInfo(u32 * setupData) {
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
			XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);

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
		print(" Searching for BoidCPUs (press ENTER to stop)...");
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
		return 0;
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

	if (channel == 1)
		getfslx(value, 1, FSL_NONBLOCKING);
	else if (channel == 0)
		getfslx(value, 0, FSL_NONBLOCKING);

	fsl_isinvalid(invalid);          	// Was there any data?
	fsl_iserror(error);					// Was there an error?

	if (error) {
		xil_printf("Error receiving data on Channel %d: %d\n\r", channel,
				error);
	}

	if (!invalid) {
		data[CMD_LEN] = value;
		xil_printf("Received data (Channel %d)\n\r", channel);
		int i = 0;

		// Handle invalid length values
		if ((data[CMD_LEN] == 0) || (data[CMD_LEN] > MAX_CMD_LEN)) {
			data[CMD_LEN] = MAX_CMD_LEN;
			print("Message has invalid length - correcting\n\r");
		}

		for (i = 0; i < data[CMD_LEN] - 1; i++) {
			if (channel == 1)
				getfslx(value, 1, FSL_NONBLOCKING);
			else if (channel == 0)
				getfslx(value, 0, FSL_NONBLOCKING);

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
	if (channel == 1)
		putfslx(value, 1, FSL_DEFAULT);
	else if (channel == 0)
		putfslx(value, 0, FSL_DEFAULT);

	fsl_isinvalid(invalid);
	fsl_iserror(error);

	if (invalid) {
		xil_printf("Warning - channel %d is full: %d\n\r", channel, value);
	}

	if (error) {
		xil_printf("Error writing data to channel %d: %d\n\r", channel, value);
	}
}
