#include "boidMaster.h"

//#define USING_TB				true	// Defined when using VHLS test bench

#define MAX_BOIDCPUS			32		// TODO: Decide on a suitable value
#define MAX_GATEKEEPERS			16		// TODO: Decide on a suitable value

#define SIMULATION_WIDTH		1920	// The pixel width of the simulation
#define SIMULATION_HEIGHT		1080	// The pixel height of the simulation

#define NORTH_IDX	12			// The index of the north edge change (load bal)
#define EAST_IDX	8			// The index of the east edge change (load bal)
#define SOUTH_IDX	4			// The index of the south edge change (load bal)
#define WEST_IDX	0			// The index of the west edge change (load bal)

// Function headers ============================================================
void processUserData();
void processPingReply();
void processAck();
void processLoadData();

void issuePing();
void issueSetupInformation();
void sendUserDataToBoidGPU();

void issueCalcNbrsMode();
void issueCalcBoidMode();
void issueLoadBalance();
void issueTransferMode();
void issueDrawMode();

void updateMinimalBoidCPUsList();
void killSimulation();

void setupSimulation();
void closestMultiples(uint12 *height, uint12 *width, uint8 number);

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
	uint12 boidCPUCoords[EDGE_COUNT];
	uint8 neighbours[MAX_BOIDCPU_NEIGHBOURS];
	uint32 gatekeeperID;
	uint8 x;
	uint8 y;
	bool minimalHeight;
	bool minimalWidth;
};

struct AckStruct {
	uint32 gatekeeperID;
	bool recieved;
	bool loadBalancing;
};

uint8 state = CMD_PING;
uint8 ackCount = 0;
AckStruct ackList[MAX_GATEKEEPERS];
uint8 gatekeeperCount = 0;
uint8 boidCPUCount = 0;
BoidCPU boidCPUs[MAX_BOIDCPUS];

uint12 simulationGridHeight = 0;
uint12 simulationGridWidth  = 0;
uint8 gridAssignment[MAX_BOIDCPUS][MAX_BOIDCPUS];

uint32 boidCount = 100;

bool continueOperation = true;

void boidMaster(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port = input
#pragma HLS INTERFACE ap_fifo port = output
#pragma HLS RESOURCE variable = input core = AXI4Stream
#pragma HLS RESOURCE variable = output core = AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port = return

// Continually check for input and deal with it. Note that reading an empty
// input stream will generate warnings in HLS, but should be blocking in the
// actual implementation.

#ifdef USING_TB
	inputData[CMD_LEN] = input.read();
#endif

	bool pingEnd = true;

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

		// STATE CHANGE --------------------------------------------------------
		if (inputData[CMD_TO] == CONTROLLER_ID) {
			switch (inputData[CMD_TYPE]) {
			case CMD_PING_START:
				pingEnd = false;
				issuePing();
				break;
			case CMD_USER_INFO:
				processUserData();
				break;
			case CMD_PING_REPLY:
				if (!pingEnd) processPingReply();
				break;
			case CMD_PING_END:
				pingEnd = true;
				break;
			case CMD_LOAD_BAL_REQUEST:
				processLoadData();
				break;
			case CMD_BOUNDS_AT_MIN:
				updateMinimalBoidCPUsList();
				break;
			case CMD_ACK:
				processAck();
				break;
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

	state = CMD_SIM_SETUP;
	setupSimulation();
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
	ackList[gatekeeperCount].gatekeeperID = inputData[CMD_FROM];
	gatekeeperCount++;

	pingResponseLoop: for (int i = 0; i < inputData[CMD_HEADER_LEN + 0]; i++) {
		boidCPUs[boidCPUCount] = BoidCPU();

		boidCPUs[boidCPUCount].gatekeeperID = inputData[CMD_FROM];
		boidCPUs[boidCPUCount].boidCPUID = FIRST_BOIDCPU_ID + boidCPUCount;
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
	uint12 boidsPerBoidCPU = boidCount / boidCPUCount;
	uint12 remainingBoids = (boidCount - (boidsPerBoidCPU * boidCPUCount));

	setupBoidCountLoop: for (int i = 0; i < boidCPUCount; i++) {
		if (i == (boidCPUCount - 1)) {
			boidCPUs[i].boidCount = boidsPerBoidCPU + remainingBoids;
		} else {
			boidCPUs[i].boidCount = boidsPerBoidCPU;
		}
	}

	// Determine simulation grid layout
	closestMultiples(&simulationGridHeight, &simulationGridWidth, boidCPUCount);

	std::cout << "Simulation is " << simulationGridWidth << " BoidCPUs wide by "
			<< simulationGridHeight << " BoidCPUs high" << std::endl;

	// Calculate coordinates
	// First, calculate the pixel width and height of one BoidCPU
	uint12 boidCPUPixelWidth = SIMULATION_WIDTH / simulationGridWidth;
	uint12 widthRemainder = (SIMULATION_WIDTH - (boidCPUPixelWidth *
			simulationGridWidth));

	uint12 boidCPUPixelHeight = SIMULATION_HEIGHT / simulationGridHeight;
	uint12 heightRemainder = (SIMULATION_HEIGHT - (boidCPUPixelHeight *
			simulationGridHeight));

	std::cout << "Typical BoidCPU dimensions: " << boidCPUPixelWidth <<
			" pixels wide by " << boidCPUPixelHeight << " pixels high" <<
			std::endl;

	// Then calculate each BoidCPU's coordinates
	uint8 count = 0;
	uint12 height = 0;
	uint12 width = 0;
	coordHeightLoop: for (int h = 0; h < simulationGridHeight; h++) {
		width = 0;

		coordWidthLoop: for (int w = 0; w < simulationGridWidth; w++) {
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

			// Is minimal?
			if ((boidCPUs[count].boidCPUCoords[2] - boidCPUs[count].\
					boidCPUCoords[0]) <= VISION_RADIUS) {
				boidCPUs[count].minimalHeight = true;
			} else if ((boidCPUs[count].boidCPUCoords[3] - boidCPUs[count].\
					boidCPUCoords[1]) <= VISION_RADIUS) {
				boidCPUs[count].minimalWidth = true;
			} else {
				boidCPUs[count].minimalHeight = false;
				boidCPUs[count].minimalWidth = false;
			}

			// Store the grid position of the BoidCPU
			boidCPUs[count].x = w;
			boidCPUs[count].y = h;

			gridAssignment[h][w] = boidCPUs[count].boidCPUID;
			count++;
			width += boidCPUPixelWidth;
		}

		height += boidCPUPixelHeight;
	}

	// Calculate neighbours
	neighbourCalcOuterLoop: for (int i = 0; i < boidCPUCount; i++) {
		uint8 x = boidCPUs[i].x;
		uint8 y = boidCPUs[i].y;

		uint8 xMinusOne = (x == 0) ? (simulationGridWidth - 1) : x - 1;
		uint8 xPlusOne  = (x == (simulationGridWidth - 1)) ? 0 : x + 1;
		uint8 yMinusOne = (y == 0) ? (simulationGridHeight - 1) : y - 1;
		uint8 yPlusOne  = (y == (simulationGridHeight - 1)) ? 0 : y + 1;

		boidCPUs[i].neighbours[0] = gridAssignment[yMinusOne][xMinusOne];
		boidCPUs[i].neighbours[1] = gridAssignment[yMinusOne][x];
		boidCPUs[i].neighbours[2] =	gridAssignment[yMinusOne][xPlusOne];
		boidCPUs[i].neighbours[3] = gridAssignment[y][xPlusOne];
		boidCPUs[i].neighbours[4] = gridAssignment[yPlusOne][xPlusOne];
		boidCPUs[i].neighbours[5] = gridAssignment[yPlusOne][x];
		boidCPUs[i].neighbours[6] = gridAssignment[yPlusOne][xMinusOne];
		boidCPUs[i].neighbours[7] = gridAssignment[y][xMinusOne];
	}

	// Calculate the number of distinct neighbours for a BoidCPU
	dNbrOut: for (int i = 0; i < boidCPUCount; i++) {
		bool isOwnNeighbour = false;
		uint8 duplicateCount = 0;
		uint8 duplicateList[MAX_BOIDCPU_NEIGHBOURS];

		// Initialise the duplicate list to 0
		for (int d = 0; d < MAX_BOIDCPU_NEIGHBOURS; d++) {
			duplicateList[d] = 0;
		}

		// Find duplicates
		dNbrMid: for (int j = 0; j < MAX_BOIDCPU_NEIGHBOURS; j++) {
			if (duplicateList[j] == 0) {
				dNbrIn: for (int k = j + 1; k < MAX_BOIDCPU_NEIGHBOURS; k++) {
					if (boidCPUs[i].neighbours[j] == boidCPUs[i].neighbours[k]) {
						duplicateList[k] = 1;
						duplicateCount++;

						if (boidCPUs[i].boidCPUID == boidCPUs[i].neighbours[j]) {
							isOwnNeighbour = true;
						}
					}
				}
			}
		}

		// Calculate number of distinct neighbours
		uint8 distinctNeighbourCount = MAX_BOIDCPU_NEIGHBOURS - duplicateCount;
		if (isOwnNeighbour) {
			distinctNeighbourCount--;
		}
		boidCPUs[i].distinctNeighbourCount = distinctNeighbourCount;
	}

	issueSetupInformation();
}

void closestMultiples(uint12 *height, uint12 *width, uint8 number) {
	uint8 difference = -1;

	incCloestMultLoop: for (int i = 1; i < number + 1; i++) {
		decClosestMultLoop: for (int j = number; j > 0; j--) {
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

/**
 * Process a received ACK. Typically this moves the simulation on to the next
 * stage. When an ACK is received mark that gatekeeper as having send its ACK
 * and increment the ACK counter.
 *
 * In load balancing, BoidCPUs send an ACK if they do not need to load balance.
 * A BoidCPU that does need to load balance will send a load balance request,
 * not an ACK. During load balancing, other BoidCPUs may be affected. Every
 * gatekeeper that has BoidCPUs affected by the load balancing has its ACK flag
 * cleared and a load balancing flag set for it. That way, when an ACK from the
 * original load balancing mode is received after the load balancing change
 * messages have been received, this ACK is only counted if the gatekeeper it
 * is from does not have any affected BoidCPUs. Otherwise, it is ignored. For
 * gatekeepers that have affected BoidCPUs, another ACK will be sent.
 */
void processAck() {
	if (inputData[CMD_FROM] == BOIDGPU_ID) {
		state = MODE_CALC_NBRS;
		issueCalcNbrsMode();
		ackCount = 0;
	} else {
		for (int i = 0; i < gatekeeperCount; i++) {
			if (ackList[i].gatekeeperID == inputData[CMD_FROM]) {

				if (ackList[i].loadBalancing == true) {
					if (inputData[CMD_HEADER_LEN] == CMD_LOAD_BAL) {
						ackList[i].recieved = true;
						ackCount++;
					} else {
						std::cout << "Ignored ACK (as load bal)" << std::endl;
					}
				} else {
					ackList[i].recieved = true;
					ackCount++;
				}
			}
		}
	}

	if (ackCount == gatekeeperCount) {
		switch(state) {
		case CMD_SIM_SETUP:
			state = MODE_CALC_NBRS;
			issueCalcNbrsMode();
			break;
		case MODE_CALC_NBRS:
			state = MODE_POS_BOIDS;
			issueCalcBoidMode();
			break;
		case MODE_POS_BOIDS:
			state = MODE_TRAN_BOIDS;
			issueTransferMode();
			break;
		case MODE_TRAN_BOIDS:
			state = MODE_LOAD_BAL;
			issueLoadBalance();
			break;
		case MODE_LOAD_BAL:
			state = MODE_DRAW;
			issueDrawMode();
			break;
		default:
			break;
		}

		ackCount = 0;
		for (int i = 0; i < gatekeeperCount; i++) {
			ackList[i].recieved = false;
			ackList[i].loadBalancing = false;
		}
	}
}

/**
 * Calculate the boundary changes for the overloaded BoidCPU and the other
 * BoidCPUs that are affected by this change. A step size is the same as the
 * vision radius of a boid.
 */
void processLoadData() {
	// TODO: Utilise the minimal BoidCPU information

	// Get info about BoidCPU making request
	uint8 x = boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].x;
	uint8 y = boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].y;

	// Determine changes for the BoidCPU that made the request
	int16 edgeChanges = 0;
	int4 stepChanges = 1;
	std::cout << "Overloaded BoidCPU (#" << inputData[CMD_FROM] << ") [" <<
			x << ", " <<  y << "]: ";

	// If the BoidCPU is not on the topmost row of the simulation grid
	if (y != 0) {
		edgeChanges |= (int16(stepChanges) << NORTH_IDX);
		std::cout << "NORTH edge decreased, ";
	}

	// If the BoidCPU is not on the rightmost column of the simulation grid
	if (x != (simulationGridWidth - 1)) {
		edgeChanges |= ((~(int16)0xF000) & (int16(-stepChanges) << EAST_IDX));
		std::cout << "EAST edge decreased, ";
	}

	// If the BoidCPU is not on the bottom-most row of the simulation grid
	if (y != (simulationGridHeight - 1)) {
		edgeChanges |= ((~(int16)0xFF00) & (int16(-stepChanges) << SOUTH_IDX));
		std::cout << "SOUTH edge decreased, ";
	}

	// If the BoidCPU is not on the leftmost column of the simulation gird
	if (x != 0) {
		edgeChanges |= ((~(int16)0xFFF0) & (int16(stepChanges) << WEST_IDX));
		std::cout << "WEST edge decreased, ";
	}
	std::cout << std::endl;

	// Determine changes for other, affected BoidCPUs
	for (int i = 0; i < boidCPUCount; i++) {

		int16 affectedBoidCPUEdgeChanges = 0;
		std::cout << "BoidCPU #" << boidCPUs[i].boidCPUID << ": ";

		// If the NORTH edge of the overloaded BoidCPU is changing and this
		// BoidCPU is on the row above the overloaded one, lower this
		// BoidCPU's SOUTH edge. If on same row lower its NORTH edge.
		if (int4(edgeChanges >> NORTH_IDX) != 0) {
			if (boidCPUs[i].y == (y - 1)) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xFF00) &
						(int16(stepChanges) << SOUTH_IDX));
				std::cout << "SOUTH edge increased, ";
			} else if (boidCPUs[i].y == y) {
				affectedBoidCPUEdgeChanges |= ((int16(stepChanges) << NORTH_IDX));
				std::cout << "NORTH edge decreased, ";
			}
		}

		// If the SOUTH edge of the overloaded BoidCPU is changing and this
		// BoidCPU is on the row below the overloaded one, raise this
		// BoidCPU's NORTH edge. If on same row raise its SOUTH edge.
		if (int4(edgeChanges >> SOUTH_IDX) != 0) {
			if (boidCPUs[i].y == (y + 1)) {
				affectedBoidCPUEdgeChanges |= ((int16(-stepChanges) << NORTH_IDX));
				std::cout << "NORTH edge increased, ";
			} else if (boidCPUs[i].y == y) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xFF00) &
						(int16(-stepChanges) << SOUTH_IDX));
				std::cout << "SOUTH edge decreased, ";
			}
		}

		// If the EAST edge of the overloaded BoidCPU is changing and this
		// BoidCPU is on the column to the right of the overloaded one, widen
		// this BoidCPU's WEST edge. If on same column narrow its EAST edge.
		if (int4(edgeChanges >> EAST_IDX) != 0) {
			if (boidCPUs[i].x == (x + 1)) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xFFF0) &
						(int16(-stepChanges) << WEST_IDX));
				std::cout << "WEST edge increased, ";
			} else if (boidCPUs[i].x == x) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xF000) &
						(int16(-stepChanges) << EAST_IDX));
				std::cout << "EAST edge decreased, ";
			}
		}

		// If the WEST edge of the overloaded BoidCPU is changing and this
		// BoidCPU is on the column to the left of the overloaded one, widen
		// this BoidCPU's EAST edge. If on same column narrow its WEST edge.
		if (int4(edgeChanges >> WEST_IDX) != 0) {
			if (boidCPUs[i].x == (x - 1)) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xF000) &
						(int16(stepChanges) << EAST_IDX));
				std::cout << "EAST edge increased, ";
			} else if (boidCPUs[i].x == x) {
				affectedBoidCPUEdgeChanges |= ((~(int16)0xFFF0) &
						(int16(stepChanges) << WEST_IDX));
				std::cout << "WEST edge decreased, ";
			}
		}

		std::cout << " [" << int4(affectedBoidCPUEdgeChanges >> NORTH_IDX) <<
				", " << int4(affectedBoidCPUEdgeChanges >> EAST_IDX) << ", "
				<< int4(affectedBoidCPUEdgeChanges >> SOUTH_IDX) << ", "
				<< int4(affectedBoidCPUEdgeChanges >> WEST_IDX) << "]"
				<< std::endl;

		if (affectedBoidCPUEdgeChanges) {
			data[0] = (uint32)affectedBoidCPUEdgeChanges;
			createCommand(1, boidCPUs[i].boidCPUID, CONTROLLER_ID, CMD_LOAD_BAL, data);

			// Clear ACK for gatekeeper owning BoidCPU
			for (int j = 0; j < gatekeeperCount; j++) {
				if (boidCPUs[i].gatekeeperID == ackList[j].gatekeeperID) {
					ackList[j].recieved = false;
					ackList[j].loadBalancing = true;
					if (ackCount) ackCount--;

					for (int i = 0; i < gatekeeperCount; i++) {
						std::cout << ackList[i].gatekeeperID << ": (" << ackList[i].recieved << ", " << ackList[i].loadBalancing << ") , ";
					} std::cout << "(" << ackCount << ")" << std::endl;

					break;
				}
			}
		}
	}
}

/**
 * When a BoidCPU reports that it is minimal, update the BoidMaster's knowledge
 * of the BoidCPU. This is used in load balancing when determining the bound
 * changes to do to balance the load.
 *
 * 0 = minimal width, 1 = minimal height, 2 = minimal height and width
 */
void updateMinimalBoidCPUsList() {
	if (inputData[CMD_HEADER_LEN]) {
		boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].minimalWidth = true;
		std::cout << "BoidCPU #" << CMD_FROM << " at minimal width" << std::endl;
	} else if (inputData[CMD_HEADER_LEN] == 1) {
		boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].minimalHeight = true;
		std::cout << "BoidCPU #" << CMD_FROM << " at minimal height" << std::endl;
	} else {
		boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].minimalHeight = true;
		boidCPUs[inputData[CMD_FROM] - FIRST_BOIDCPU_ID].minimalWidth = true;
		std::cout << "BoidCPU #" << CMD_FROM << " at minimum" << std::endl;
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
			data[CMD_SETUP_BNBRS_IDX + j] = boidCPUs[i].neighbours[j];
		}

		data[CMD_SETUP_SIMWH_IDX + 0] = SIMULATION_WIDTH;
		data[CMD_SETUP_SIMWH_IDX + 1] = SIMULATION_HEIGHT;

		dataLength = 17;
		to = boidCPUs[i].gatekeeperID;
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

void issueLoadBalance() {
	to = CMD_BROADCAST;
	dataLength = 0;
	createCommand(dataLength, to, from, MODE_LOAD_BAL, data);
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
			std::cout << "-> TX, BoidMaster sent broadcast:                  ";
		} else if (data[CMD_TO] == BOIDGPU_ID) {
			std::cout << "-> TX, BoidMaster sent command to BoidGPU:         ";
		} else {
			std::cout << "-> TX, BoidMaster sent command to " << data[CMD_TO]
					<< ":              ";
		}
	} else {
		if (data[CMD_TO] == CMD_BROADCAST) {
			// This should never happen - only the controller can broadcast
			std::cout << "<- RX, BoidMaster received broadcast from "
					<< data[CMD_FROM] << ":      ";
		} else if (data[CMD_FROM] == BOIDGPU_ID) {
			// This should never happen - BoidGPU should just receive
			std::cout << "<- RX, BoidMaster received command from BoidGPU:   ";
		} else {
			std::cout << "<- RX, BoidMaster received command from "
					<< data[CMD_FROM] << ":        ";
		}
	}

	switch (data[CMD_TYPE]) {
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
		std::cout << "user info                         ";
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
	case MODE_LOAD_BAL:
		std::cout << "load balance mode                 ";
		break;
	case CMD_LOAD_BAL:
		std::cout << "load balance instructions         ";
		break;
	case CMD_LOAD_BAL_REQUEST:
		std::cout << "load balance request              ";
		break;
	case CMD_BOUNDS_AT_MIN:
		std::cout << "BoidCPU at minimal bounds         ";
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
	case CMD_ACK:
		std::cout << "ACK signal                        ";
		break;
	case CMD_PING_END:
		std::cout << "end of ping                       ";
		break;
	case CMD_PING_START:
		std::cout << "start of ping                     ";
		break;
	case CMD_KILL:
		std::cout << "kill simulation                   ";
		break;
	case CMD_DEBUG:
		std::cout << "debug information                 ";
		break;
	default:
		std::cout << "UNKNOWN COMMAND: (" << data[CMD_TYPE] << ")             ";
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
