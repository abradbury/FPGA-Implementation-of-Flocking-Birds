#include "xparameters.h"

#include <stdio.h>                  // For simple input/output (I/O)
#include <stdlib.h>
#include <string.h>                 // For memset()
#include "xuartlite_l.h"            // UART
#include "fsl.h"                    // AXI Steam

#define ENTER                   0x0A// The ASCII code for '\n' or Line Feed (LF)
#define USING_VLAB              0   // 1 if using VLAB, 0 if not

#define BOID_COUNT              20  // The total number of system boids

// Command definitions ---------------------------------------------------------
#define CMD_HEADER_LEN          4   // The length of the command header
#define MAX_CMD_BODY_LEN        30  // The max length of the command body
#define MAX_CMD_LEN             CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define MAX_OUTPUT_CMDS         15  // The number of output commands to buffer
#define MAX_INPUT_CMDS          5   // The number of input commands to buffer

#define CMD_LEN                 0   // The index of the command length
#define CMD_TO                  1   // The index of the command target
#define CMD_FROM                2   // The index of the command sender
#define CMD_TYPE                3   // The index of the command type

#define CMD_BROADCAST           0   // The number for a broadcast command

#define CONTROLLER_ID           1   // The ID of the controller
#define BOIDGPU_ID              2   // The ID of the BoidGPU
#define FIRST_BOIDCPU_ID        3   // The lowest possible BoidCPU ID

#define MODE_INIT               1   //
#define CMD_PING                2   // Controller -> BoidCPU
#define CMD_PING_REPLY          3   // BoidCPU -> Controller
#define CMD_USER_INFO           4   // Controller -> BoidGPU
#define CMD_SIM_SETUP           5   // Controller -> Boid[CG]PU
#define MODE_CALC_NBRS          6   //
#define CMD_NBR_REPLY           8   // BoidCPU -> BoidCPU
#define MODE_POS_BOIDS          9   //
#define CMD_LOAD_BAL            10  // TODO: Decide on implementation
#define MODE_TRAN_BOIDS         11  //
#define CMD_BOID                12  // BoidCPU -> BoidCPU
#define MODE_DRAW               14  // TODO: Perhaps not needed?
#define CMD_DRAW_INFO           15  // BoidCPU -> BoidGPU
#define CMD_KILL                16  // Controller -> All

#define CMD_COUNT               16

// BoidCPU definitions ---------------------------------------------------------
#define EDGE_COUNT              4   // The number of edges a BoidCPU has
#define MAX_BOIDCPU_NEIGHBOURS  8   // The maximum neighbours a BoidCPUs has
#define MAX_SYSTEM_BOIDCPUS     10  // The maximum number of BoidCPUs

typedef enum {
	false, true
} bool;

u32 data[MAX_CMD_BODY_LEN];
u32 to;
u32 from = CONTROLLER_ID;
u32 dataLength = 0;
u32 coords[EDGE_COUNT];

u32 inputData[MAX_CMD_LEN];

u16 boidCPUs[MAX_SYSTEM_BOIDCPUS];
u16 boidCPUCount = 0;

const char *commandDescriptions[CMD_COUNT] = { "Initialisation mode",
		"Send ping to BoidCPUs", "Ping response from a BoidCPU",
		"User-inputed information for the BoidGPU",
		"Simulation setup information for a BoidCPU",
		"Calculate neighbours mode", "",
		"Reply of neighbouring BoidCPU's boids", "Position calculation mode",
		"Load balancing command", "Transfer boids mode", "Transmit a boid", "",
		"Draw mode", "Draw information heading to BoidGPU", "Kill simulation" };

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data, int channel);
void chooseCommand(u8 commandID);

void testInitMode();
void testPing();
void testPingReply();
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
void processPingReply(u32 *data);
void processNeighbourReply(u32 *data);

void printCommand(bool send, u32 *data, int channel);

int main() {
	// TODO: setup Ethernet

	do {
		print("--------------------------------------------------\n\r");
		print("-------- FPGA Flocking Bird Test Harness ---------\n\r");
		print("--------------------------------------------------\n\r");

		bool cIDValid = false;	// True if the command ID entered is valid
		u8 cID = 0;             // The ID of the command to issue

		u8 index = 0;           // Index for keyPresses
		char keyPress;          // The key pressed
		char keyPresses[] = ""; // An array containing the keys pressed

		while (!cIDValid) {		// Ask for a valid command ID
			index = 0;          // Reset the index and key press array
			memset(keyPresses, 0, sizeof(keyPresses));

			// Print options
			print("Choose a command from the following list: \n\r");
			int i = 0;
			for (i = 0; i < CMD_COUNT; i++) {
				xil_printf(" %d: %s\n\r", i + 1, commandDescriptions[i]);
			}
			print("--------------------------------------------------\n\r");

			// Take keyboard input until the ENTER key is pressed
			do {
				// While there is no keyboard input, check for received messages
				int rvOne, rvTwo, invalidOne, invalidTwo;
				do {
					getfslx(rvOne, 0, FSL_NONBLOCKING);	// Check for data (c0)
					fsl_isinvalid(invalidOne);          // Was there any data?

					getfslx(rvTwo, 1, FSL_NONBLOCKING);	// Check for data (c0)
					fsl_isinvalid(invalidTwo);			// Was there any data?

					if (!invalidOne) {
						inputData[CMD_LEN] = rvOne;
						xil_printf("Received data (Channel 0) of length %d \n\r", inputData[CMD_LEN]);
						int i = 0, value = 0;

						// Handle invalid length values
						if ((inputData[CMD_LEN] == 0) || (inputData[CMD_LEN] > MAX_CMD_LEN)) {
							inputData[CMD_LEN] = MAX_CMD_LEN;
							print("Message has invalid length - correcting\n\r");
						}

						for (i = 0; i < inputData[CMD_LEN] - 1; i++) {
							getfslx(value, 0, FSL_NONBLOCKING);
							inputData[i + 1] = value;
						}
						processResponse(inputData, 0);
					}

					if (!invalidTwo) {
						print("Received data (Channel 1)\n\r");
						inputData[CMD_LEN] = rvTwo;
						int i = 0, value = 0;

						// Handle invalid length values
						if ((inputData[CMD_LEN] == 0) || (inputData[CMD_LEN] > MAX_CMD_LEN)) {
							inputData[CMD_LEN] = MAX_CMD_LEN;
							print("Message has invalid length - correcting\n\r");
						}

						for (i = 0; i < inputData[CMD_LEN] - 1; i++) {
							getfslx(value, 1, FSL_NONBLOCKING);
							inputData[i + 1] = value;
						}
						processResponse(inputData, 1);
					}

				} while (XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR));

				keyPress = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);
				if (USING_VLAB)
					XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);

				keyPresses[index] = keyPress;
				index++;

			} while (keyPress != ENTER);     // Repeat while enter isn't pressed

			print("Enter key press registered\n\r");

			cID = (u8) atoi(keyPresses); // Convert the key pressed to int, 0 if not int

			xil_printf("Received command %d\n\r", cID);

			if ((cID >= 1) && (cID <= 16)) {
				print("Command ID is valid\n\r");
				cIDValid = true;
				chooseCommand(cID);				// Send the command
			} else {
				print("\n\r**Error: Command ID must be between 1 and "\
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
	case MODE_INIT:
		testInitMode();
		break;
	case CMD_PING:
		testPing();
		break;
	case CMD_PING_REPLY:
		testPingReply();
		break;
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

void testInitMode() {
	// 4, 0, 1, 1 ||
	u32 random_seed = 0x1871abc8;
	to = CMD_BROADCAST;
	dataLength = 1;

	int i;
	for(i = 0; i < 2; i++) {
		data[0] = (u16)(random_seed/(i+1));
		createCommand(dataLength, to, from, MODE_INIT, data, i);
	}
}

void testPing() {
	// Test ping response ----------------------------------------------------//
	// 4, 0, 1, 2 ||
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, CMD_PING, data, -1);
}

void testPingReply() {
	// 6, 1, 42, 3 || 21, 123
	to = CONTROLLER_ID;
	from = 42;

	data[0] = 21;
	data[1] = 123;
	dataLength = 2;

	createCommand(dataLength, to, from, CMD_PING_REPLY, data, -1);
}

void testUserInfo() {
	// 7, 2, 1, 4 || 21 42 84
	to = BOIDGPU_ID;
	data[0] = 21;
	data[1] = 42;
	data[2] = 84;
	dataLength = 3;

	createCommand(dataLength, to, from, CMD_USER_INFO, data, -1);
}

// TODO: Re-do this to make it actually extendible
void testSimulationSetup() {
	// Setup stuff
	u32 initialBoidCount = 10;
	u32 neighbours[MAX_BOIDCPU_NEIGHBOURS];
	int i = 0;
	dataLength = 14;

	u32 newIDOne = 0 + FIRST_BOIDCPU_ID;
	u32 newIDTwo = 1 + FIRST_BOIDCPU_ID;

	xil_printf("%d BoidCPUs discovered\n\r", boidCPUCount);

	// BoidCPU1
	if (boidCPUCount == 1) {
		to = boidCPUs[0];
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
		for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			data[EDGE_COUNT + 2 + i] = neighbours[i];
		}
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data, 0);
	}

	// BoidCPU2
	if (boidCPUCount == 2) {
		to = boidCPUs[0];
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
		for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			data[EDGE_COUNT + 2 + i] = neighbours[i];
		}
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data, 0);

		//--

		to = boidCPUs[1];
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
		for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			data[EDGE_COUNT + 2 + i] = neighbours[i];
		}
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data, 1);
	}
}

void testNeighbourSearch() {
	// 4 0 1 6 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_CALC_NBRS, data, -1);
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

	createCommand(dataLength, to, from, CMD_NBR_REPLY, data, -1);
}

void testCalcNextBoidPos() {
	// 4 0 1 9 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_POS_BOIDS, data, -1);
}

void testLoadBalance() {
	// 4 0 1 10 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, CMD_LOAD_BAL, data, -1);
}

void testMoveBoids() {
	// 4 0 1 11 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_TRAN_BOIDS, data, -1);
}

void testBoidCommand() {
	dataLength = 5;
	to = CMD_BROADCAST;

	data[0] = 42;	// ID
	data[1] = 48;	// x pos
	data[2] = 20;	// y pos
	data[3] = -1;	// x vel
	data[4] = -3;	// y vel

	createCommand(dataLength, to, from, CMD_BOID, data, -1);

	data[0] = 43;	// ID
	data[1] = 53;	// x pos
	data[2] = 21;	// y pos
	data[3] = 0;	// x vel
	data[4] = 4;	// y vel

	createCommand(dataLength, to, from, CMD_BOID, data, -1);
}

void testDrawBoids() {
	// 4 0 1 14 ||
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_DRAW, data, -1);
}

void testDrawInfo() {
	print("Testing draw info - TODO\n\r");
}

void testKillSwitch() {
	print("Testing kill switch - TODO\n\r");
}

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data, int channel) {
//	print("Creating command...");
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
//	print("done\n\r");

//	print("Sending command...");
	int i = 0;
	for (i = 0; i < CMD_HEADER_LEN + len; i++) {
		// FSLX ID cannot be a variable - currently transmitting on all
		// channels without checking if the addressee is on that channel
		if(channel == 0) {
			putfslx(command[i], 0, FSL_NONBLOCKING);
		} else if(channel == 1) {
			putfslx(command[i], 1, FSL_NONBLOCKING);
		} else {
			putfslx(command[i], 0, FSL_NONBLOCKING);
			putfslx(command[i], 1, FSL_NONBLOCKING);
		}
	}
//	print("done\n\r");

	printCommand(true, command, channel);
}

//============================================================================//
//- Command Processing -------------------------------------------------------//
//============================================================================//

void processResponse(u32 *data, u8 channel) {
	printCommand(false, data, channel);

	if((data[CMD_TO] >= FIRST_BOIDCPU_ID) && (data[CMD_TYPE] <= CMD_COUNT)) {
		int dataLength = data[CMD_LEN] - CMD_HEADER_LEN;
		u32 dataToForward[dataLength];

		int i = 0;
		for(i = 0; i < dataLength; i++) {
			dataToForward[i] = data[CMD_HEADER_LEN + i];
		}

		xil_printf("Controller forwarding command to BoidCPU #%d\n\r", \
				data[CMD_TO]);

		if (channel == 0) {
			createCommand(dataLength, data[CMD_TO], data[CMD_FROM],
				data[CMD_TYPE], dataToForward, 1);
		} else if (channel == 1) {
			createCommand(dataLength, data[CMD_TO], data[CMD_FROM],
				data[CMD_TYPE], dataToForward, 0);
		}
	}

	switch (data[CMD_TYPE]) {
	case CMD_PING_REPLY:
		processPingReply(data);
		break;
	case CMD_NBR_REPLY:
		processNeighbourReply(data);
		break;
	default:
		break;
	}
}

/**
 * Get the ID of the responder and add it to a list of BoidCPUs
 * TODO: Store the FPGA ID as well
 */
void processPingReply(u32 *data) {
	boidCPUs[boidCPUCount] = data[CMD_FROM];
	boidCPUCount++;
}

void processNeighbourReply(u32 *data) {
	//TODO
}

//============================================================================//
//- Command Debug ------------------------------------------------------------//
//============================================================================//

void printCommand(bool send, u32 *data, int channel) {
	if (send) {
		if (data[CMD_TO] == CMD_BROADCAST) {
			print("-> TX, Controller sent broadcast");
		} else if (data[CMD_TO] == BOIDGPU_ID) {
			print("-> TX, Controller sent command to BoidGPU");
		} else {
			xil_printf("-> TX, Controller sent command to %d", data[CMD_TO]);
		}
	} else {
		if (data[CMD_TO] == CMD_BROADCAST) {
			// This should never happen - BoidCPUs should not be able to broadcast
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
		print("do something");
		break;
	case MODE_INIT:
		print("initialise self");
		break;
	case CMD_PING:
		print("BoidCPU ping");
		break;
	case CMD_PING_REPLY:
		print("BoidCPU ping response");
		break;
	case CMD_USER_INFO:
		print("output user info");
		break;
	case CMD_SIM_SETUP:
		print("setup BoidCPU");
		break;
	case MODE_CALC_NBRS:
		print("calculate neighbours");
		break;
	case CMD_NBR_REPLY:
		print("neighbouring boids from neighbour");
		break;
	case MODE_POS_BOIDS:
		print("calculate new boid positions");
		break;
	case CMD_LOAD_BAL:
		print("load balance");
		break;
	case MODE_TRAN_BOIDS:
		print("transfer boids");
		break;
	case CMD_BOID:
		print("boid in transit");
		break;
	case MODE_DRAW:
		print("send boids to BoidGPU");
		break;
	case CMD_DRAW_INFO:
		print("boid info heading to BoidGPU");
		break;
	case CMD_KILL:
		print("kill simulation");
		break;
	default:
		print("UNKNOWN COMMAND");
		break;
	}

	XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, '\t');
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

