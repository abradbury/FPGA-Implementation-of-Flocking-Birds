#include "xparameters.h"

#include <stdio.h>                  // For simple input/output (I/O)
#include <stdlib.h>
#include <string.h>                 // For memset()
#include "xuartlite_l.h"            // UART
#include "fsl.h"                    // AXI Steam
#include "boids.h"

#define BOIDCPU_CHANNEL_1		1
#define BOIDCPU_CHANNEL_2		2
#define ALL_BODICPU_CHANNELS	-1

u32 data[MAX_CMD_BODY_LEN];
u32 to;
u32 from = CONTROLLER_ID;
u32 dataLength = 0;
u32 coords[EDGE_COUNT];

u32 gatekeeperID;
u16 boidCmdCount = 0;
u16 boidCPUCount = 2;

const char *commandDescriptions[CMD_COUNT] = { "", "", "",
		"User-inputed information for the BoidGPU",
		"Simulation setup information for a BoidCPU",
		"Calculate neighbours mode", "",
		"Reply of neighbouring BoidCPU's boids", "Position calculation mode",
		"Load balancing command", "Transfer boids mode", "Transmit a boid", "",
		"Draw mode", "Draw information heading to BoidGPU", "Kill simulation" };

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data, int channel);
void chooseCommand(u8 commandID);

void testUserInfo();
void testSimulationSetup();
void testNeighbourSearch();
void testNeighbourReply();
void testCalcNextBoidPos();
void testLoadBalance();
void testMoveBoids();
void testBoidCommand();
void testDrawBoids();
void testDrawInfo();
void testKillSwitch();

void processResponse(u32 *data, u8 channel);

void printCommand(bool send, u32 *data, int channel);

void putData(u32 value, u32 channel);
u32 getData(u32 *data, u32 channel);

int main() {
	do {
		print("--------------------------------------------------\n\r");
		print("-------- FPGA Flocking Bird Test Harness ---------\n\r");
		print("--------------------------------------------------\n\r");

		gatekeeperID = rand();	// Random ID of the Gatekeeper
		bool cIDValid = false;	// True if the command ID entered is valid
		u8 cID = 0;             // The ID of the command to issue

		u8 index = 0;           // Index for keyPresses
		char keyPress;          // The key pressed
		char keyPresses[] = ""; // An array containing the keys pressed

		while (!cIDValid) {		// Ask for a valid command ID
			index = 0;          // Reset the index and key press array
			memset(keyPresses, 0, sizeof(keyPresses));

			// Print options
			int i = 0;
			for (i = 0; i < CMD_COUNT; i++) {
				xil_printf(" %3d: %s\n\r", i + 1, commandDescriptions[i]);
			}
			print("--------------------------------------------------\n\r");
			print("Enter a command from the above list: ");

			// Take keyboard input until the ENTER key is pressed
			do {
				// While there is no keyboard input, check for received messages
				do {
					// Channel 1 -----------------------------------------------
					u32 channelOneData[MAX_CMD_LEN];
					int channelOneInvalid = getData(channelOneData,
							BOIDCPU_CHANNEL_1);
					// ---------------------------------------------------------

					// Channel 2 -----------------------------------------------
					u32 channelTwoData[MAX_CMD_LEN];
					int channelTwoInvalid = getData(channelTwoData,
							BOIDCPU_CHANNEL_2);
					// ---------------------------------------------------------

					// Process received data -----------------------------------
					if (!channelOneInvalid) {
						processResponse(channelOneData, BOIDCPU_CHANNEL_1);
					}

					if (!channelTwoInvalid) {
						processResponse(channelTwoData, BOIDCPU_CHANNEL_2);
					}
					// ---------------------------------------------------------

				} while (XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR));

				keyPress = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);
				if (USING_VLAB) {
					XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);
				}

				keyPresses[index] = keyPress;
				index++;

			} while (keyPress != ENTER);     // Repeat while enter isn't pressed

			// Convert the key pressed to int, 0 if not int
			cID = (u8) atoi(keyPresses);

			if ((cID >= 1) && (cID <= 16)) {
				cIDValid = true;
				chooseCommand(cID);				// Send the command
			} else {
				print("\n\r**Error: Command ID must be between 1 and "
						"16 inclusive. Please try again.\n\r");
			}
		}
	} while (1);

	return 0;
}

//============================================================================//
//- Command Generation -------------------------------------------------------//
//============================================================================//

void chooseCommand(u8 commandID) {
	from = CONTROLLER_ID;

	switch (commandID) {
	case CMD_USER_INFO:
		testUserInfo();
		break;
	case CMD_SIM_SETUP:
		testSimulationSetup();
		break;
	case MODE_CALC_NBRS:
		testNeighbourSearch();
		break;
	case CMD_NBR_REPLY:
		testNeighbourReply();
		break;
	case MODE_POS_BOIDS:
		testCalcNextBoidPos();
		break;
	case CMD_LOAD_BAL:
		testLoadBalance();
		break;
	case MODE_TRAN_BOIDS:
		testMoveBoids();
		break;
	case CMD_BOID:
		testBoidCommand();
		break;
	case MODE_DRAW:
		testDrawBoids();
		break;
	case CMD_DRAW_INFO:
		testDrawInfo();
		break;
	case CMD_KILL:
		testKillSwitch();
		break;
	default:
		print("UNKNOWN COMMAND\n\r");
		break;
	}
}

void testUserInfo() {
	// 7, 2, 1, 4 || 21 42 84
	to = BOIDGPU_ID;
	data[0] = 21;
	data[1] = 42;
	data[2] = 84;
	dataLength = 3;

	createCommand(dataLength, to, from, CMD_USER_INFO, data,
			ALL_BODICPU_CHANNELS);
}

// TODO: Re-do this to make it actually extendible
void testSimulationSetup() {
	// Setup stuff
	u32 initialBoidCount = 10;
	u32 neighbours[MAX_BOIDCPU_NEIGHBOURS];
	int i = 0;
	dataLength = 17;

	u32 distinctNeighbourCount = 1;
	u32 newIDOne = 0 + FIRST_BOIDCPU_ID;
	u32 newIDTwo = 1 + FIRST_BOIDCPU_ID;

	xil_printf("Gatekeeper %d is responsible for %d BoidCPUs\n\r", gatekeeperID,
			boidCPUCount);

	// BoidCPU2
	if (boidCPUCount == 2) {
		to = CMD_BROADCAST;
		coords[0] = 0;
		coords[1] = 0;
		coords[2] = 40;
		coords[3] = 40;
		neighbours[0] = newIDTwo;
		neighbours[1] = newIDOne;
		neighbours[2] = newIDTwo;
		neighbours[3] = newIDTwo;
		neighbours[4] = newIDTwo;
		neighbours[5] = newIDOne;
		neighbours[6] = newIDTwo;
		neighbours[7] = newIDTwo;

		data[0] = newIDOne;
		data[1] = initialBoidCount;
		for (i = 0; i < EDGE_COUNT; i++) {
			data[2 + i] = coords[i];
		}
		data[2 + EDGE_COUNT] = distinctNeighbourCount;
		for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			data[2 + EDGE_COUNT + 1 + i] = neighbours[i];
		}
		data[CMD_SETUP_SIMWH_IDX + 0] = 80;
		data[CMD_SETUP_SIMWH_IDX + 1] = 40;
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data,
				BOIDCPU_CHANNEL_1);

		//--

		to = CMD_BROADCAST;
		coords[0] = 40;
		coords[1] = 0;
		coords[2] = 80;
		coords[3] = 40;
		neighbours[0] = newIDOne;
		neighbours[1] = newIDTwo;
		neighbours[2] = newIDOne;
		neighbours[3] = newIDOne;
		neighbours[4] = newIDOne;
		neighbours[5] = newIDTwo;
		neighbours[6] = newIDOne;
		neighbours[7] = newIDOne;

		data[0] = newIDTwo;
		data[1] = initialBoidCount;
		for (i = 0; i < EDGE_COUNT; i++) {
			data[2 + i] = coords[i];
		}
		data[2 + EDGE_COUNT] = distinctNeighbourCount;
		for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			data[2 + EDGE_COUNT + 1 + i] = neighbours[i];
		}
		data[CMD_SETUP_SIMWH_IDX + 0] = 80;
		data[CMD_SETUP_SIMWH_IDX + 1] = 40;
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data,
				BOIDCPU_CHANNEL_2);
	}
}

void testNeighbourSearch() {
	// 4 0 1 6 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_CALC_NBRS, data,
			ALL_BODICPU_CHANNELS);
}

void testNeighbourReply() {
	dataLength = 6;
	to = CMD_BROADCAST;

	data[0] = 212990464;	// position
	data[1] = 1049856;		// velocity
	data[2] = 101;			// ID
	data[3] = 204574464;
	data[4] = 3144448;
	data[5] = 102;

	createCommand(dataLength, to, from, CMD_NBR_REPLY, data,
			ALL_BODICPU_CHANNELS);
}

void testCalcNextBoidPos() {
	// 4 0 1 9 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_POS_BOIDS, data,
			ALL_BODICPU_CHANNELS);
}

void testLoadBalance() {
	// 4 0 1 10 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, CMD_LOAD_BAL, data,
			ALL_BODICPU_CHANNELS);
}

void testMoveBoids() {
	// 4 0 1 11 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_TRAN_BOIDS, data,
			ALL_BODICPU_CHANNELS);
}

void testBoidCommand() {
	dataLength = 5;
	to = CMD_BROADCAST;

	data[0] = 42 + boidCmdCount;	// ID
	data[1] = 48;	// x pos
	data[2] = 20;	// y pos
	data[3] = -1;	// x vel
	data[4] = -3;	// y vel

	createCommand(dataLength, to, from, CMD_BOID, data, ALL_BODICPU_CHANNELS);

	data[0] = 43 + boidCmdCount;	// ID
	data[1] = 53;	// x pos
	data[2] = 21;	// y pos
	data[3] = 0;	// x vel
	data[4] = 4;	// y vel

	createCommand(dataLength, to, from, CMD_BOID, data, ALL_BODICPU_CHANNELS);

	boidCmdCount++;
}

void testDrawBoids() {
	// 4 0 1 14 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_DRAW, data, ALL_BODICPU_CHANNELS);
}

void testDrawInfo() {
	print("Testing draw info - TODO\n\r");
}

void testKillSwitch() {
	print("Testing kill switch - TODO\n\r");
}

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data, int channel) {
	u32 command[MAX_CMD_LEN];

	command[CMD_LEN] = len + CMD_HEADER_LEN;
	command[CMD_TO] = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		int i = 0;
		for (i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}

	int i = 0;
	for (i = 0; i < CMD_HEADER_LEN + len; i++) {
		// FSLX ID cannot be a variable - currently transmitting on all
		// channels without checking if the addressee is on that channel
		if (channel == 0) {
			print("WARNING: Channel 0 is now the controller...\n\r");
//			putData(command[i], 0);
		} else if (channel == BOIDCPU_CHANNEL_1) {
			putData(command[i], BOIDCPU_CHANNEL_1);
		} else if (channel == BOIDCPU_CHANNEL_2) {
			putData(command[i], BOIDCPU_CHANNEL_2);
		} else {
//			putData(command[i], 0);
			putData(command[i], BOIDCPU_CHANNEL_1);
			putData(command[i], BOIDCPU_CHANNEL_2);
		}
	}

	printCommand(true, command, channel);
}

void putData(u32 value, u32 channel) {
	int error = 0, invalid = 0;

	// TODO: Ensure that writes use FSL_DEFAULT (blocking write)
	if (channel == BOIDCPU_CHANNEL_2)
		putfslx(value, BOIDCPU_CHANNEL_2, FSL_DEFAULT);
	else if (channel == BOIDCPU_CHANNEL_1)
		putfslx(value, BOIDCPU_CHANNEL_1, FSL_DEFAULT);
//	else if (channel == 0) putfslx(value, 0, FSL_DEFAULT);

	fsl_isinvalid(invalid);
	fsl_iserror(error);

	if (invalid) {
		xil_printf("Warning - channel %d is full: %d\n\r", channel, value);
	}

	if (error) {
		xil_printf("Error writing data to channel %d: %d\n\r", channel, value);
	}
}

u32 getData(u32 *data, u32 channel) {
	int invalid, error, value = 0;

	if (channel == BOIDCPU_CHANNEL_2)
		getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
	else if (channel == BOIDCPU_CHANNEL_1)
		getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
//	else if (channel == 0) getfslx(value, 0, FSL_NONBLOCKING);

	fsl_isinvalid(invalid);// Was there any data?
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
			if (channel == BOIDCPU_CHANNEL_2)
				getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
			else if (channel == BOIDCPU_CHANNEL_1)
				getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
			//	else if (channel == 0) getfslx(value, 0, FSL_NONBLOCKING);

			fsl_iserror(error);// Was there an error?
			if (error) {
				xil_printf("Error receiving data on Channel %d: %d\n\r",
						channel, error);
			}

			data[i + 1] = value;
		}
	}

	return invalid;
}

//============================================================================//
//- Command Processing -------------------------------------------------------//
//============================================================================//

void processResponse(u32 *data, u8 channel) {
	printCommand(false, data, channel);

	if ((data[CMD_TO] >= FIRST_BOIDCPU_ID) && (data[CMD_TYPE] <= CMD_COUNT)) {
		int dataLength = data[CMD_LEN] - CMD_HEADER_LEN;
		u32 dataToForward[dataLength];

		int i = 0;
		for (i = 0; i < dataLength; i++) {
			dataToForward[i] = data[CMD_HEADER_LEN + i];
		}

		if (channel == 0) {
			print("WARNING: Received message from channel 0...\n\r");
		} else if (channel == BOIDCPU_CHANNEL_1) {
			createCommand(dataLength, data[CMD_TO], data[CMD_FROM],
					data[CMD_TYPE], dataToForward, BOIDCPU_CHANNEL_2);
		} else if (channel == BOIDCPU_CHANNEL_2) {
			createCommand(dataLength, data[CMD_TO], data[CMD_FROM],
					data[CMD_TYPE], dataToForward, BOIDCPU_CHANNEL_1);
		}
	} // else {
//		switch (data[CMD_TYPE]) {
//		case CMD_NBR_REPLY:
//			processNeighbourReply(data);
//			break;
//		default:
//			break;
//		}
//	}
}

//============================================================================//
//- Command Debug ------------------------------------------------------------//
//============================================================================//

void printCommand(bool send, u32 *data, int channel) {
	if (send) {
		if (data[CMD_TO] == CMD_BROADCAST) {
			print("-> TX, Controller sent broadcast                 ");
		} else if (data[CMD_TO] == BOIDGPU_ID) {
			print("-> TX, Controller sent command to BoidGPU      ");
		} else {
			xil_printf("-> TX, Controller sent command to %d      ",
					data[CMD_TO]);
		}
	} else {
		if (data[CMD_TO] == CMD_BROADCAST) {
			// This should never happen - BoidCPUs cannot broadcast
			xil_printf("<- RX, Controller received broadcast from %d",
					data[CMD_FROM]);
		} else if (data[CMD_FROM] == BOIDGPU_ID) {
			// This should never happen
			print("<- RX, Controller received command from BoidGPU");
		} else {
			xil_printf("<- RX, Controller received command from %d",
					data[CMD_FROM]);
		}
	}

	xil_printf(" (on channel %d): ", channel);

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
		break;
	case CMD_ACK:
		print("ACK signal                        ");
		break;
	case CMD_KILL:
		print("kill simulation                   ");
		break;
	default:
		print("UNKNOWN COMMAND                   ");
		break;
	}

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

