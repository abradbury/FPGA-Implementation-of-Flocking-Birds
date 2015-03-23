#include "boidMaster.h"

#include <iostream>     // cout
#include <math.h>       // sqrt, floor

#define USING_TB		true
#define MAX_BOIDCPUS	32		// TODO: Decide on a suitable value

// Function headers ============================================================
void processPingReply();

void issuePing();
void issueSetupInformation();
void sendUserDataToBoidGPU();

void issueCalcNbrsMode();
void issueCalcBoidMode();
//void issueLoadBalance(); // TODO
void issueTransferMode();
void issueDrawMode();

void killSimulation();

void setupSimulation();
void closestMultiples(uint8 *height, uint8 *width, uint8 number);

void printCommand(bool send, uint32 *data);
void createCommand(uint32 len, uint32 to, uint32 from, uint32 type,
		uint32 *data);

// Global variables ============================================================
uint32 outputData[MAX_OUTPUT_CMDS][MAX_CMD_LEN];
uint32 inputData[MAX_CMD_LEN];
uint32 outputCount = 0;

uint32 data[MAX_CMD_BODY_LEN];
uint32 to;
uint32 from = CONTROLLER_ID;
uint32 dataLength = 0;

struct BoidCPU {
	uint8 boidCPUID;
	uint8 boidCount;
	uint8 distinctNeighbourCount;
	uint8 boidCPUCoords[EDGE_COUNT];
	uint8 neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS];
	uint8 gatekeeperID;
};

uint8 boidCPUCount = 0;
BoidCPU boidCPUs[MAX_BOIDCPUS];

uint32 boidCount = 100;

bool continueOperation = true;

void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port = input
#pragma HLS INTERFACE ap_fifo port = output
#pragma HLS RESOURCE variable = input core = AXI4Stream
#pragma HLS RESOURCE variable = output core = AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port = return

	// Perform initialisation
//    initialisation();

// Continually check for input and deal with it. Note that reading an empty
// input stream will generate warnings in HLS, but should be blocking in the
// actual implementation.

#ifdef USING_TB
	inputData[CMD_LEN] = input.read();
#endif

	mainWhileLoop: while (continueOperation) {
		// INPUT ---------------------------------------------------------------
		// Block until there is input available
#ifndef USING_TB
		inputData[CMD_LEN] = input.read();
#endif
		// When there is input, read in the command
		inputLoop: for (int i = 1; i < inputData[CMD_LEN]; i++) {
			inputData[i] = input.read();
		}
		printCommand(false, inputData);
		// ---------------------------------------------------------------------

		/**
		 * Power on system
		 *
		 * Controller waits for user information (from the MicroBlaze)
		 * Stores this
		 *
		 * Pings boidCPUs
		 * Processes ping replies as they arrive
		 * Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * Calculates simulation setup information
		 * Issues setup command to BoidGPU
		 * Issues setup commands to each BoidCPU
		 * Waits for a certain time - TODO: How does it know how long to wait?
		 * 		Waits for all ACKs?
		 *
		 * Begins main simulation loop
		 * - Issues calculate neighbours mode
		 * - Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * - Issues position calculation mode
		 * - Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * - Issues load balancing command
		 * - Process load balancing replies
		 * - Performs load balancing
		 * - Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * - Issues transfer mode
		 * - Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * - Issues draw command - TODO: Is this needed?
		 * - Waits for a certain time - TODO: How does it know how long to wait?
		 *
		 * - Repeats
		 */

		// STATE CHANGE --------------------------------------------------------
		if (inputData[CMD_TO] == CONTROLLER_ID) {
			switch (inputData[CMD_TYPE]) {
			case CMD_PING_REPLY:
				processPingReply();

				// TODO: Remove
				setupSimulation();
				//

				break;
//                case CMD_LOAD_BAL_REPLY:
//                	// TODO: Implement load balancing
//                    processLoadData();
//                    break;
			default:
				std::cout << "Command state " << inputData[CMD_TYPE]
						<< " not recognised" << std::endl;
				break;
			}
		} else {
			std::cout << "The above message was ignored" << std::endl;
		}
		// ---------------------------------------------------------------------

		// OUTPUT --------------------------------------------------------------
		// If there is output to send, send it
		if (outputCount > 0) {
			outerOutLoop: for (int j = 0; j < outputCount; j++) {
				innerOutLoop: for (int i = 0; i < outputData[j][CMD_LEN]; i++) {
					output.write(outputData[j][i]);
				}
				printCommand(true, outputData[j]);
			}
		}
		outputCount = 0;
		// ---------------------------------------------------------------------

#ifdef USING_TB
		continueOperation = input.read_nb(inputData[0]);
#endif
	}
	std::cout << "=========BoidMaster has finished=========" << std::endl;
}

//============================================================================//
// Incoming functions --------------------------------------------------------//
//============================================================================//

void processUserData() {
	// TODO: Create user -> boidMaster process
	boidCount = inputData[CMD_HEADER_LEN + 0];
}

/**
 * A ping reply will be sent by the Gatekeeper responsible for the BoidCPUs it
 * serves. The reply will contain the number of BoidCPUs that are served by the
 * Gatekeeper.
 *
 * First, a BoidCPU is created for each BoidCPU served by the Gatekeeper. These
 * created BoidCPUs are linked with the Gatekeeper's ID and given an ID based
 * on their index in the BoidCPU array.
 *
 * Then, when all the ping replies have been received (how to know this is
 * another matter), the simulation setup is calculated.
 */
void processPingReply() {
	uint8 gatekeeprBoidCPUCount = inputData[CMD_HEADER_LEN + 0];
	for (int i = 0; i < gatekeeprBoidCPUCount; i++) {
		boidCPUs[boidCPUCount] = BoidCPU();

		boidCPUs[boidCPUCount].gatekeeperID = inputData[CMD_FROM];
		boidCPUs[boidCPUCount].boidCPUID = boidCPUCount;
		boidCPUCount++;
	}
}

/**
 * Sets up the simulation based on the discovered BoidCPUs.
 *
 * First, the initial boid counts for each BoidCPU is calculated and supplied
 * to the BoidCPUs. If the division of boids is not whole, any remaining boids
 * are assigned to the last BoidCPU.
 *
 * Then the grid height and width of the simulation area is determined. This is
 * done by finding the closest two multiples of the boidCPU count. The larger
 * multiple is assigned to the width as most monitors are wider than tall.
 *
 * Then, the coordinates are defined.
 *
 * Finally, neighbours are supplied. Ideally those BoidCPUs served by the same
 * Gatekeeper will be placed as neighbours.
 */
void setupSimulation() {
	// Define initial boids counts
	uint16 boidsPerBoidCPU = boidCount / boidCPUCount;
	uint16 remainingBoids = (boidCount - (boidsPerBoidCPU * boidCPUCount));

	for (int i = 0; i < boidCPUCount; i++) {
		if (i == (boidCPUCount - 1)) {
			boidCPUs[i].boidCount = boidsPerBoidCPU + remainingBoids;
		} else {
			boidCPUs[i].boidCount = boidsPerBoidCPU;
		}
	}

	// Global ---------------
	uint16 simulationHeight = 0;
	uint16 simulationWidth  = 0;
	// ----------------------

	// Determine simulation grid layout
	uint8 simulationGridHeight = 0;
	uint8 simulationGridWidth  = 0;

	closestMultiples(&simulationGridHeight, &simulationGridWidth, boidCPUCount);

	// Calculate coordinates
	// First, calculate the pixel width and height of one BoidCPU
	uint16 boidCPUPixelWidth = simulationWidth / simulationGridWidth;
	uint16 widthRemainder = (simulationWidth - (boidCPUPixelWidth *
			simulationGridWidth));

	uint16 boidCPUPixelHeight = simulationHeight / simulationGridHeight;
	uint16 heightRemainder = (simulationHeight - (boidCPUPixelHeight *
			simulationGridHeight));

	// Then calculate each BoidCPU's coordinates
	uint16 count = 0;
	uint16 height = 0;
	for (int h = 0; h < simulationGridHeight; h++) {
		uint16 width = 0;

		for (int w = 0; w < simulationGridWidth; w++) {
			boidCPUs[count].boidCPUCoords[0] = width;
			boidCPUs[count].boidCPUCoords[1] = height;

			// If the last column, add any area width division remainder
			if (w == (simulationGridWidth - 1)) {
				boidCPUs[count].boidCPUCoords[2] = boidCPUs[count].\
						boidCPUCoords[0] + boidCPUPixelWidth + widthRemainder;
			} else {
				boidCPUs[count].boidCPUCoords[2] = boidCPUs[count].\
						boidCPUCoords[0] + boidCPUPixelWidth;
			}

			// If the last row, add any area height division remainder
			if (h == (simulationGridHeight - 1)) {
				boidCPUs[count].boidCPUCoords[3] = boidCPUs[count].\
						boidCPUCoords[1] + boidCPUPixelHeight + heightRemainder;
			} else {
				boidCPUs[count].boidCPUCoords[3] = boidCPUs[count].\
						boidCPUCoords[1] + boidCPUPixelHeight;
			}

			count++;
			width += boidCPUPixelWidth;
		}

		height += boidCPUPixelHeight;
	}

	// Calculate neighbours - TODO
//	for (int i = 0; i < boidCPUCount; i++) {
//		for (int j = 0; j < MAX_BOIDCPU_NEIGHBOURS; j++) {
//
//		}
//	}

	//    neighbours[0] = 4;
	//    neighbours[1] = 3;
	//    neighbours[2] = 4;
	//    neighbours[3] = 4;
	//    neighbours[4] = 4;
	//    neighbours[5] = 3;
	//    neighbours[6] = 4;
	//    neighbours[7] = 4;
}

void closestMultiples(uint8 *height, uint8 *width, uint8 number) {
	uint8 difference = -1;

	incCloestMultLoop: for (int i = 1; i < number + 1; i++) {
		decClosestMultLoop: for (int j = number; j > 0; j++) {
			if (i > j) break;
			if ((i * j) == number) {
				if ((j - i) < difference) {
					*height = (uint8)i;
					*width = (uint8)j;
					difference = j - i;
				}
			}
		}
	}
}

//============================================================================//
// Outgoing functions --------------------------------------------------------//
//============================================================================//

void issuePing() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, CMD_PING, data);
}

void issueSetupInformation() {
	for(int i = 0; i < boidCPUCount; i++) {
		data[CMD_SETUP_NEWID_IDX] = boidCPUs[i].boidCPUID;
		data[CMD_SETUP_BDCNT_IDX] = boidCPUs[i].boidCount;

		for (int j = 0; j < EDGE_COUNT; j++) {
			data[CMD_SETUP_COORD_IDX + j] = boidCPUs[i].boidCPUCoords[j];
		}

		data[CMD_SETUP_NBCNT_IDX] = boidCPUs[i].distinctNeighbourCount;

		for (int j = 0; j < MAX_BOIDCPU_NEIGHBOURS; j++) {
			data[CMD_SETUP_BNBRS_IDX + j] = boidCPUs[i].neighbouringBoidCPUs[j];
		}

		dataLength = 15;
		createCommand(dataLength, to, from, CMD_SIM_SETUP, data);
	}
}

void sendUserDataToBoidGPU() {
	// TODO: Implement graphics
}

void issueCalcNbrsMode() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, MODE_CALC_NBRS, data);
}

void issueCalcBoidMode() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, MODE_POS_BOIDS, data);
}

void issueTransferMode() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, MODE_TRAN_BOIDS, data);
}

void issueDrawMode() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, MODE_DRAW, data);
}

void killSimulation() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, CMD_KILL, data);
}

//============================================================================//
// Message processing functions ----------------------------------------------//
//============================================================================//

void createCommand(uint32 len, uint32 to, uint32 from, uint32 type,
		uint32 *data) {
	outputData[outputCount][CMD_LEN] = len + CMD_HEADER_LEN;
	outputData[outputCount][CMD_TO] = to;
	outputData[outputCount][CMD_FROM] = from;
	outputData[outputCount][CMD_TYPE] = type;

	if (len > 0) {
		dataToCmd: for (int i = 0; i < len; i++) {
			outputData[outputCount][CMD_HEADER_LEN + i] = data[i];
		}
	}

	outputCount++;
}

//============================================================================//
// Debug ---------------------------------------------------------------------//
//============================================================================//

void printCommand(bool send, uint32 *data) {
	if (send) {
		if (data[CMD_TO] == CMD_BROADCAST) {
			std::cout << "-> TX, Controller sent broadcast:                  ";
		} else if (data[CMD_TO] == BOIDGPU_ID) {
			std::cout << "-> TX, Controller sent command to BoidGPU:       ";
		} else {
			std::cout << "-> TX, Controller sent command to " << data[CMD_TO]
					<< ":       ";
		}
	} else {
		if (data[CMD_TO] == CMD_BROADCAST) {
			// This should never happen - only the controller can broadcast
			std::cout << "<- RX, Controller received broadcast from "
					<< data[CMD_FROM] << ": ";
		} else if (data[CMD_FROM] == BOIDGPU_ID) {
			// This should never happen - BoidGPU should just receive
			std::cout << "<- RX, Controller received command from BoidGPU: ";
		} else {
			std::cout << "<- RX, Controller received command from "
					<< data[CMD_FROM] << ": ";
		}
	}

	switch (data[CMD_TYPE]) {
	case 0:
		std::cout << "do something                      ";
		break;
	case MODE_INIT:
		std::cout << "initialise self                   ";
		break;
	case CMD_PING:
		std::cout << "BoidCPU ping                      ";
		break;
	case CMD_PING_REPLY:
		std::cout << "BoidCPU ping response             ";
		break;
	case CMD_USER_INFO:
		std::cout << "output user info                  ";
		break;
	case CMD_SIM_SETUP:
		std::cout << "setup BoidCPU                     ";
		break;
	case MODE_CALC_NBRS:
		std::cout << "calculate neighbours              ";
		break;
	case CMD_NBR_REPLY:
		std::cout << "neighbouring boids from neighbour ";
		break;
	case MODE_POS_BOIDS:
		std::cout << "calculate new boid positions      ";
		break;
	case CMD_LOAD_BAL:
		std::cout << "load balance                      ";
		break;
	case MODE_TRAN_BOIDS:
		std::cout << "transfer boids                    ";
		break;
	case CMD_BOID:
		std::cout << "boid in transit                   ";
		break;
	case MODE_DRAW:
		std::cout << "send boids to BoidGPU             ";
		break;
	case CMD_DRAW_INFO:
		std::cout << "boid info heading to BoidGPU      ";
		break;
	case CMD_KILL:
		std::cout << "kill simulation                   ";
		break;
	default:
		std::cout << "UNKNOWN COMMAND                   ";
		break;
	}

	int i = 0;
	for (i = 0; i < CMD_HEADER_LEN; i++) {
		std::cout << data[i] << " ";
	}
	std::cout << "|| ";

	for (i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
		std::cout << data[CMD_HEADER_LEN + i] << " ";
	}
	std::cout << std::endl;
}
