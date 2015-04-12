#include "boidCPU.h"

//#define USING_TB	true

// Function headers ============================================================
// Key function headers --------------------------------------------------------
static void simulationSetup(void);
static void calcNextBoidPositions(void);
static void evaluateLoad(void);
static void loadBalance(void);
static void calculateEscapedBoids(void);
static void updateDisplay(void);

void calculateBoidNeighbours(void);
void sendBoidsToNeighbours(void);
void processNeighbouringBoids(void);

// Supporting function headers -------------------------------------------------
void transmitBoids(uint16 *boidIDs, uint8 *recipientIDs, uint8 count);
void acceptBoid();

void packBoidsForSending(uint32 to, uint32 msg_type);
Boid parsePackedBoid(uint8 offset);

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data);
bool fromNeighbour();

void commitAcceptedBoids();

bool isBoidBeyond(Boid boid, uint8 edge);
bool isBoidBeyondSingle(Boid boid, uint8 edge);
bool isNeighbourTo(uint16 bearing);

// Debugging function headers --------------------------------------------------
void printCommand(bool send, uint32 *data);
void printStateOfBoidCPUBoids();


// Global variables ============================================================
// TODO: Determine which variables should really be global
// BoidCPU variables -----------------------------------------------------------
int8 boidCPUID = FIRST_BOIDCPU_ID;
int12 boidCPUCoords[4];

uint11 simulationWidth  = 0;
uint11 simulationHeight = 0;

uint8 neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS];
bool neighbouringBoidCPUsSetup = false;	// True when neighbouring BoidCPUs setup

uint8 distinctNeighbourCount = 0;		// The number of distinct neighbouring BoidCPUs
uint8 distinctNeighbourCounter = 0;		// A counter to the above

int16 queuedBoids[MAX_QUEUED_BOIDS][5];// Holds boids received from neighbours
uint8 queuedBoidsCounter = 0;			// A counter for queued boids

uint32 inputData[MAX_CMD_LEN];
uint32 outputData[MAX_OUTPUT_CMDS][MAX_CMD_LEN];
uint32 outputBody[30];
uint8 outputCount = 0;             		// The number of output messages stored

// Boid variables --------------------------------------------------------------
uint8 boidCount;
Boid boids[MAX_BOIDS];     // TODO: Perhaps re-implement as a LL due to deletion

// TODO: Try with and without pointers (for synthesising)
Boid *boidNeighbourList[MAX_BOIDS][MAX_NEIGHBOURING_BOIDS];

// A list of possible neighbouring boids for the BoidCPU
Boid possibleBoidNeighbours[MAX_NEIGHBOURING_BOIDS];
uint8 possibleNeighbourCount = 0;	// Number of possible boid neighbours

// Debugging variables ---------------------------------------------------------
bool continueOperation = true;


void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
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
        if ((inputData[CMD_FROM] != boidCPUID) &&
        		((inputData[CMD_TO] == boidCPUID) ||
        				(inputData[CMD_TO] == CMD_BROADCAST) ||
        				fromNeighbour())) {

            switch (inputData[CMD_TYPE]) {
                case CMD_SIM_SETUP:
                    simulationSetup();
                    break;
                case MODE_CALC_NBRS:
                    sendBoidsToNeighbours();
                    break;
                case CMD_NBR_REPLY:
                    processNeighbouringBoids();
                    break;
                case MODE_POS_BOIDS:
                    calcNextBoidPositions();
                    break;
                case MODE_LOAD_BAL:
                    evaluateLoad();
                    break;
                case CMD_LOAD_BAL:
                	loadBalance();
                	break;
                case MODE_TRAN_BOIDS:
                    calculateEscapedBoids();
                    break;
                case CMD_BOID:
                	acceptBoid();
                	break;
                case MODE_DRAW:
                    updateDisplay();
                    break;
                default:
                    std::cout << "Command state " << inputData[CMD_TYPE] <<
                        " not recognised" << std::endl;
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
    std::cout << "=============BoidCPU has finished==============" << std::endl;
}

//==============================================================================
// State functions =============================================================
//==============================================================================

/**
 * Sets ID to that provided by the controller
 * Populates list of neighbouring BoidCPUs
 * Initialises pixel coordinates and edges
 * Creates own boids
 */
void simulationSetup() {
    std::cout << "-Preparing BoidCPU for simulation..." << std::endl;

    // Set BoidCPU parameters (supplied by the controller)
    int8 oldBoidCPUID = boidCPUID;
    boidCPUID = inputData[CMD_HEADER_LEN + CMD_SETUP_NEWID_IDX];
    boidCount = inputData[CMD_HEADER_LEN + CMD_SETUP_BDCNT_IDX];

    edgeSetupLoop: for (int i = 0; i < EDGE_COUNT; i++) {
        boidCPUCoords[i] = inputData[CMD_HEADER_LEN + CMD_SETUP_COORD_IDX + i];
    }

    // Get the number of distinct neighbours
    distinctNeighbourCount = inputData[CMD_HEADER_LEN + CMD_SETUP_NBCNT_IDX];

    // Get the list of neighbours on each edge
    neighbourSetupLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        neighbouringBoidCPUs[i] =
            inputData[CMD_HEADER_LEN + CMD_SETUP_BNBRS_IDX + i];
    }
    neighbouringBoidCPUsSetup = true;

    // Get the simulation width and height
    simulationWidth  = inputData[CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX];
    simulationHeight = inputData[CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX + 1];

    // Print out BoidCPU parameters
    std::cout << "BoidCPU #" << oldBoidCPUID << " now has ID #" <<
        boidCPUID << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " initial boid count: " <<
        boidCount << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " has " << distinctNeighbourCount
    	<< " distinct neighbouring BoidCPUs" << std::endl;

    std::cout << "BoidCPU #" << boidCPUID << " coordinates: [";
    printEdgeLoop: for (int i = 0; i < EDGE_COUNT; i++) {
        std::cout << boidCPUCoords[i] << ", ";
    } std::cout << "]" << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " neighbours: [";
    printNeighbourLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        std::cout << neighbouringBoidCPUs[i] << ", ";
    } std::cout << "]" << std::endl;

    std::cout << "The simulation is of width " << simulationWidth <<
    	" and of height " << simulationHeight << std::endl;

    // Create the boids (actual implementation)
//  uint16 boidID;
//  boidCreationLoop: for(int i = 0; i < boidCount; i++) {
//      Vector velocity = Vector(getRandom(-MAX_VELOCITY, MAX_VELOCITY),
//          getRandom(-MAX_VELOCITY, MAX_VELOCITY));
//      Vector position = Vector(getRandom(boidCPUCoords[X_MIN],
//          boidCPUCoords[X_MAX]), getRandom(boidCPUCoords[Y_MIN],
//          boidCPUCoords[Y_MAX]));
//
//      boidID = ((boidCPUID - 1) * boidCount) + i + 1;
//
//      Boid boid = Boid(boidID, position, velocity, i);
//      boids[i] = boid;
//  }

    // Create the boids (testing implementation)
//    uint8 testBoidCount = boidCount;
    Vector knownSetup[10][2] = {{Vector(12, 11), Vector(5, 0)},
            {Vector(19.3, 35), Vector(-5, 1.76)},
            {Vector(12, 31), Vector(-4, -2)},
            {Vector(35, 22), Vector(0, -3)},
            {Vector(4, 9), Vector(-1, 0)},
            {Vector(19, 18), Vector(2, -3)},
            {Vector(38, 19), Vector(4, -4)},
            {Vector(18, 5), Vector(-1, 2)},
            {Vector(15, 33), Vector(2, -2)},
            {Vector(3, 8), Vector(-2, 0)}};

    uint16 baseBoidID = 0;
    setupMultLoop: for (int i = 0; i < (boidCPUID - 1); i++) {
    	baseBoidID += boidCount;
    }

    testBoidCreationLoop: for (int i = 0; i < boidCount; i++) {
        uint16 boidID = baseBoidID + i + 1;
        Boid boid = Boid(boidID, knownSetup[i][0], knownSetup[i][1]);
        boids[i] = boid;
    }

    // Send ACK signal
    outputBody[0] = CMD_SIM_SETUP;
    generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
}

/**
 * Send this BoidCPU's boids to its neighbouring BoidCPUs so they can use them
 * in calculating neighbours for their boids. Splits the data to be sent into
 * multiple messages if it exceeds the maximum command data size.
 */
void sendBoidsToNeighbours() {
    std::cout << "-Sending boids to neighbouring BoidCPUs..." << std::endl;

	packBoidsForSending(CMD_MULTICAST, CMD_NBR_REPLY);
}

/**
 * When boids are received from neighbouring BoidCPUs, process them and add
 * them to a list of possible neighbouring boids. These are used to calculate
 * the neighbours of boids contained within this BoidCPU.
 */
void processNeighbouringBoids() {
	// Before processing first response, add own boids to list
	if (distinctNeighbourCounter == 0) {
		addOwnBoidsToNbrList: for (int i = 0; i < boidCount; i++) {
			possibleBoidNeighbours[possibleNeighbourCount] = boids[i];
			possibleNeighbourCount++;
		}
	}

    // Calculate the number of boids per message TODO: Remove division
	uint8 boidsPerMsg = (inputData[CMD_LEN] - CMD_HEADER_LEN - 1) /
			BOID_DATA_LENGTH;

    std::cout << "-BoidCPU #" << boidCPUID << " received " << boidsPerMsg <<
        " boids from BoidCPU #" << inputData[CMD_FROM] << std::endl;

    // Parse each received boid and add to possible neighbour list
    rxNbrBoidLoop: for (int i = 0; i < boidsPerMsg; i++) {
        possibleBoidNeighbours[possibleNeighbourCount] = parsePackedBoid(i);

        // Don't go beyond the edge of the array
        if (possibleNeighbourCount != (MAX_NEIGHBOURING_BOIDS - 1)) {
        	possibleNeighbourCount++;
    	}
    }

    // If no further messages are expected, then process it
    if (inputData[CMD_HEADER_LEN + 0] == 0) {
    	distinctNeighbourCounter++;

    	if (distinctNeighbourCounter == distinctNeighbourCount) {
			calculateBoidNeighbours();

			// Send ACK signal
			outputBody[0] = MODE_CALC_NBRS;
			outputBody[1] = distinctNeighbourCounter;
			outputBody[2] = distinctNeighbourCount;
			generateOutput(3, CONTROLLER_ID, CMD_ACK, outputBody);
		}
    } else {
    	std::cout << "Expecting " << inputData[CMD_HEADER_LEN + 0] << \
    			" further message(s) from " << inputData[CMD_FROM] << std::endl;
    }
}

/**
 * This is called when the BoidCPUs are instructed to calculate the next
 * positions of the boids. It was in the neighbour search stage, but it became
 * complex to determine when all the boids from neighbours had been received
 * due to splitting of data over multiple messages and replicated neighbours.
 */
void calculateBoidNeighbours() {
	outerCalcBoidNbrsLoop: for (int i = 0; i < boidCount; i++) {
		uint8 boidNeighbourCount = 0;
		inCalcBoidNbrsLoop: for (int j = 0; j < possibleNeighbourCount; j++) {
			if (possibleBoidNeighbours[j].id != boids[i].id) {
				int16_fp boidSeparation = Vector::squaredDistanceBetween(
					boids[i].position, possibleBoidNeighbours[j].position);

				if (boidSeparation < VISION_RADIUS_SQUARED) {
					boidNeighbourList[i][boidNeighbourCount] =
						&possibleBoidNeighbours[j];
					boidNeighbourCount++;
				}
			}
		}

		boids[i].setNeighbourDetails(i, boidNeighbourCount);
	}

	// Reset the flags
	possibleNeighbourCount = 0;
	distinctNeighbourCounter = 0;
}

void calcNextBoidPositions() {
    std::cout << "-Calculating next boid positions..." << std::endl;

    updateBoidsLoop: for (int i = 0; i < boidCount; i++) {
        boids[i].update();

        // Contain boid pixel position values to within the simulation area
        if (boids[i].position.x > simulationWidth) {
        	boids[i].position.x = 0;
        } else if (boids[i].position.x < 0) {
        	boids[i].position.x = simulationWidth;
        }

        if (boids[i].position.y > simulationHeight) {
        	boids[i].position.y = 0;
        } else if (boids[i].position.y < 0) {
        	boids[i].position.y = simulationHeight;
        }
    }

    // Send ACK signal
    outputBody[0] = MODE_POS_BOIDS;
	generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
}

// If overloaded, signal the controller
void evaluateLoad() {
    if (boidCount > BOID_THRESHOLD) {
    	std::cout << "-Load balancing..." << std::endl;

    	generateOutput(0, CONTROLLER_ID, CMD_LOAD_BAL_REQUEST, outputBody);
    } else {
    	std::cout << "-No need to load balance" << std::endl;
    	outputBody[0] = MODE_LOAD_BAL;
		generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
    }
}

void loadBalance() {
	int16 edgeChanges = (int16)inputData[CMD_HEADER_LEN + 0];

	std::cout << "BoidCPU #" << boidCPUID << " changing NORTH edge from " <<
			boidCPUCoords[Y_MIN];
	boidCPUCoords[Y_MIN] += VISION_RADIUS * int4(edgeChanges >> NORTH_IDX);
	std::cout << " to " << boidCPUCoords[Y_MIN] << std::endl;

	std::cout << "BoidCPU #" << boidCPUID << " changing EAST edge from " <<
			boidCPUCoords[X_MAX];
	boidCPUCoords[X_MAX] += VISION_RADIUS * int4(edgeChanges >> EAST_IDX);
	std::cout << " to " << boidCPUCoords[X_MAX] << std::endl;

	std::cout << "BoidCPU #" << boidCPUID << " changing SOUTH edge from " <<
			boidCPUCoords[Y_MAX];
	boidCPUCoords[Y_MAX] += VISION_RADIUS * int4(edgeChanges >> SOUTH_IDX);
	std::cout << " to " << boidCPUCoords[Y_MAX] << std::endl;

	std::cout << "BoidCPU #" << boidCPUID << " changing WEST edge from " <<
			boidCPUCoords[X_MIN];
	boidCPUCoords[X_MIN] += VISION_RADIUS * int4(edgeChanges >> WEST_IDX);
	std::cout << " to " << boidCPUCoords[X_MIN] << std::endl;

	// Is minimal?
	int12 width  = boidCPUCoords[2] - boidCPUCoords[0];
	int12 height = boidCPUCoords[3] - boidCPUCoords[1];
	if ((width <= VISION_RADIUS) && (height <= VISION_RADIUS)) {
		std::cout << "BoidCPU #" << boidCPUID << " minimal" << std::endl;
		outputBody[0] = 2;
		generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
	} else if (width <= VISION_RADIUS) {
		std::cout << "BoidCPU #" << boidCPUID << " width minimal" << std::endl;
		outputBody[0] = 0;
		generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
	} else if (height <= VISION_RADIUS) {
		std::cout << "BoidCPU #" << boidCPUID << " height minimal" << std::endl;
		outputBody[0] = 1;
		generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
	}
}

void updateDisplay() {
	if (queuedBoidsCounter > 0) {
		commitAcceptedBoids();

		std::cout << "-Updating display" << std::endl;
		packBoidsForSending(BOIDGPU_ID, CMD_DRAW_INFO);
	} else {
		std::cout << "-Updating display" << std::endl;
		packBoidsForSending(BOIDGPU_ID, CMD_DRAW_INFO);
	}
}

//==============================================================================
//- Boid Transmission and Acceptance -------------------------------------------
//==============================================================================

void calculateEscapedBoids() {
    std::cout << "-Transferring boids..." << std::endl;

    uint16 boidIDs[MAX_BOIDS];
	uint8 recipientIDs[MAX_BOIDS];
	uint8 counter = 0;

	// For each boid
	moveBoidsLoop: for (int i = 0; i < boidCount; i++) {
		// For each bearing (from NORTHWEST (0) to WEST (7))
		bearLoop: for (int bearing = NORTHWEST; bearing < WEST + 1; bearing++) {
			// If a BoidCPU is at the bearing & boid is beyond the bearing limit
			if (isNeighbourTo(bearing) && isBoidBeyond(boids[i], bearing)) {
				// Mark boid as to be transferred
				boidIDs[counter] = boids[i].id;
				recipientIDs[counter] = neighbouringBoidCPUs[bearing];
				counter++;
			}
		}
	}

	if (counter > 0) {
		transmitBoids(boidIDs, recipientIDs, counter);
	} else {
		// Send ACK signal
		outputBody[0] = MODE_TRAN_BOIDS;
		generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
	}
}

/**
 * Checks if the supplied boid is beyond the supplied BoidCPU edge. Can handle
 * compound edge bearings such as NORTHWEST.
 */
bool isBoidBeyond(Boid boid, uint8 edge) {
	bool result;

	switch (edge) {
	case NORTHWEST:
		result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
		break;
	case NORTHEAST:
		result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
		break;
	case SOUTHEAST:
		result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
		break;
	case SOUTHWEST:
		result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
		break;
	default:
		result = isBoidBeyondSingle(boid, edge);
		break;
	}

	return result;
}

/**
 * Checks if the supplied boid is beyond the supplied BoidCPU edge. Can only
 * handle singular edge bearings e.g. NORTH.
 */
bool isBoidBeyondSingle(Boid boid, uint8 edge) {
	int16_fp coordinate;
	bool result;

	switch (edge){
	case NORTH:
		edge = Y_MIN;
		break;
	case EAST:
		edge = X_MAX;
		break;
	case SOUTH:
		edge = Y_MAX;
		break;
	case WEST:
		edge = X_MIN;
		break;
	default:
		break;
	}

	// Determine the coordinate to check against
	if ((edge == X_MIN) || (edge == X_MAX)) {
		coordinate = boid.position.x;
	} else if ((edge == Y_MIN) || (edge == Y_MAX)) {
		coordinate = boid.position.y;
	}

	// Determine if the comparison should be less than or greater than
	if ((edge == X_MIN) || (edge == Y_MIN)) {
		if (coordinate < boidCPUCoords[edge]) {
			result = true;
		} else {
			result = false;
		}
	} else if ((edge == X_MAX) || (edge == Y_MAX)) {
		if (coordinate > boidCPUCoords[edge]) {
			result = true;
		} else {
			result = false;
		}
	}

	return result;
}

/**
 * Checks if the BoidCPU has a neighbour at the specified bearing
 */
bool isNeighbourTo(uint16 bearing) {
	if (neighbouringBoidCPUs[bearing] > 0) {
		return true;
	} else {
		return false;
	}
}

void transmitBoids(uint16 *boidIDs, uint8 *recipientIDs, uint8 count) {
	// First transmit all the boids
	boidTransmitLoop: for (int i = 0; i < count; i++) {
		boidTransmitSearchLoop: for (int j = 0; j < boidCount; j++) {
			if (boidIDs[i] == boids[j].id) {
				// TODO: Perhaps move this to the boid class?
				outputBody[0] = boids[j].id;
				outputBody[1] = boids[j].position.x;
				outputBody[2] = boids[j].position.y;
				outputBody[3] = boids[j].velocity.x;
				outputBody[4] = boids[j].velocity.y;

				generateOutput(5, recipientIDs[i], CMD_BOID, outputBody);

				std::cout << "-Transferring boid #" << boids[j].id <<
					" to boidCPU #" << recipientIDs[i] << std::endl;

				break;
			}
		}
	}

	// Then delete the boids from the BoidCPUs own boid list
	outerBoidRemovalLoop: for (int i = 0; i < count; i++) {
		bool boidFound = false;
		//  j < boidCount - 1 as list is decremented by 1
		// TODO: Avoid using indexes as these will change - use IDs
		innerBoidRemovalLoop: for (int j = 0; j < boidCount - 1; j++) {
			if (boids[j].id == boidIDs[i]) {
				boidFound = true;
			}

			if (boidFound) {
				boids[j] = boids[j + 1];
			}
		}
		boidCount--;
	}

	// Send ACK signal
	outputBody[0] = MODE_TRAN_BOIDS;
	generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
}

void acceptBoid() {
	// TODO: Replace 5 with BOID_DATA_LENGTH when using common transmission
	for (int i = 0; i < 5; i++) {
		queuedBoids[queuedBoidsCounter][i] = inputData[CMD_HEADER_LEN + i];
	}

	queuedBoidsCounter++;
}

void commitAcceptedBoids() {
	std::cout << "-Committing accepted boids..." << std::endl;

	for (int i = 0; i < queuedBoidsCounter; i++) {
		// TODO: Replace 5 with BOID_DATA_LENGTH when using common transmission
		uint16 boidID = queuedBoids[i][0];
		Vector boidPosition = Vector(queuedBoids[i][1], queuedBoids[i][2]);
		Vector boidVelocity = Vector(queuedBoids[i][3], queuedBoids[i][4]);
		Boid boid = Boid(boidID, boidPosition, boidVelocity);

		boids[boidCount] = boid;
		boidCount++;

		std::cout << "-BoidCPU #" << boidCPUID << " accepted boid #" << boidID
			<< " from boidCPU #" << inputData[CMD_FROM] << std::endl;
	}

	queuedBoidsCounter = 0;
}

//============================================================================//
//- Load Balancing Functions -------------------------------------------------//
//============================================================================//

//============================================================================//
//- Supporting functions -----------------------------------------------------//
//============================================================================//

void printStateOfBoidCPUBoids() {
    boidStatePrintLoop: for (int i = 0; i < boidCount; i++) {
        std::cout << "Boid " << boids[i].id << " has position [" <<
            boids[i].position.x << ", " << boids[i].position.y <<
            "] and velocity [" << boids[i].velocity.x << ", " <<
            boids[i].velocity.y << "]" << std::endl;
    }
}

Boid parsePackedBoid(uint8 offset) {
	uint8 index = CMD_HEADER_LEN + (BOID_DATA_LENGTH * offset);

	uint32 pos = inputData[index + 1];
	uint32 vel = inputData[index + 2];
	uint16 bID = inputData[index + 3];		// boid ID

	// Decode position and velocity
	Vector position = Vector(((int32_fp)((int32)pos >> 16)) >> 4,
			((int32_fp)((int16)pos)) >> 4);

	Vector velocity = Vector(((int32_fp)((int32)vel >> 16)) >> 4,
			((int32_fp)((int16)vel)) >> 4);

	std::cout << "-BoidCPU #" << boidCPUID << " received boid #" << bID <<
			" from BoidCPU #" << inputData[CMD_FROM] << std::endl;

	return Boid(bID, position, velocity);
}

void packBoidsForSending(uint32 to, uint32 msg_type) {
	if (boidCount > 0) {
		// The first bit of the body is used to indicate the number of messages
		uint16 partialMaxCmdBodyLen = MAX_CMD_BODY_LEN - 1;

		// First, calculate how many messages need to be sent
		// Doing this division saves a DSP at the expense of about 100 LUTs
		int16 numerator = boidCount * BOID_DATA_LENGTH;
		uint16 msgCount = 0;
		nbrMsgCountCalcLoop: for (msgCount = 0; numerator > 0; msgCount++) {
			numerator -= partialMaxCmdBodyLen;
		}

		// Then calculate the number of boids that can be sent per message
		uint16 boidsPerMsg = (uint16)(partialMaxCmdBodyLen / BOID_DATA_LENGTH);

		// Determine the initial boid indexes for this message
		uint8 startBoidIndex = 0;
		uint8 endBoidIndex   = startBoidIndex + boidsPerMsg;

		// Next, send a message for each group of boids
		nbrMsgSendLoop: for (int i = 0; i < msgCount; i++) {
			// Limit the end index if the message won't be full
			if (endBoidIndex > boidCount) {
				endBoidIndex = boidCount;
			}

			// Put the number of subsequent messages in the first body field
			outputBody[0] = msgCount - i - 1;

			// The next step is to create the message data
			uint8 index = 1;
			NMClp: for (int j = startBoidIndex, k = 0; j < endBoidIndex; j++, k++) {
				uint32 position = 0;
				uint32 velocity = 0;

				// Encode the boid position and velocity -----------------------
				// First, cast the int16_fp value to an int32_fp value. This
				// enables up to 24 bits of integer values (and 8 fractional
				// bits). Then, shift the value left by 4 bits to bring the
				// fractional bits into the integer bit range. This is needed
				// because casting an int16_fp straight to an integer causes
				// the fractional bits to be lost. After casting to a uint32
				// (the transmission data type) shift the x value to the top
				// 16 bits of the variable. The y value is placed in the bottom
				// 16 bits of the variable, but if this is negative a mask
				// needs to be applied to clear the 1s in the top 16 bits that
				// are there due to the 2s complement notation for negatives.

				// Encode position
				position |= ((uint32)(((int32_fp)(boids[j].position.x)) << 4) << 16);

				if (boids[j].position.y < 0) {
					position |= ((~(((uint32)0xFFFF) << 16)) &
							((uint32)((int32_fp)(boids[j].position.y) << 4)));
				} else {
					position |= ((uint32)((int32_fp)(boids[j].position.y) << 4));
				}

				// Encode velocity
				velocity |= ((uint32)(((int32_fp)(boids[j].velocity.x)) << 4) << 16);

				if (boids[j].velocity.y < 0) {
					velocity |= ((~(((uint32)0xFFFF) << 16)) &
							((uint32)((int32_fp)(boids[j].velocity.y) << 4)));
				} else {
					velocity |= ((uint32)((int32_fp)(boids[j].velocity.y) << 4));
				}

				outputBody[index + 0] = position;
				outputBody[index + 1] = velocity;
				// ID can be removed on deployment
				outputBody[index + 2] = boids[j].id;

				index += BOID_DATA_LENGTH;
			}

			// Finally send the message
			uint32 dataLength = (((endBoidIndex - startBoidIndex)) * BOID_DATA_LENGTH) + 1;
			generateOutput(dataLength, to, msg_type, outputBody);

			// Update the boid indexes for the next message
			startBoidIndex += boidsPerMsg;
			endBoidIndex = startBoidIndex + boidsPerMsg;
		}
	} else {
		std::cout << "No boids to send, sending empty message" << std::endl;
		outputBody[0] = 0;
		generateOutput(1, to, msg_type, outputBody);
	}
}

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data) {
	if (outputCount > MAX_OUTPUT_CMDS - 1) {
		std::cout << "Cannot send message, output buffer is full (" <<
			outputCount << "/" << MAX_OUTPUT_CMDS << ")" << std::endl;
	} else {
		outputData[outputCount][CMD_LEN]  = len + CMD_HEADER_LEN;
		outputData[outputCount][CMD_TO]   = to;
		outputData[outputCount][CMD_FROM] = boidCPUID;
		outputData[outputCount][CMD_TYPE] = type;

		if (len > 0) {
			createOutputCommandLoop: for (int i = 0; i < len; i++) {
				outputData[outputCount][CMD_HEADER_LEN + i] = data[i];
			}
		}
		outputCount++;
	}
}

/**
 * Iterate through the list of neighbouring BoidCPUs to determine whether the
 * message received was from one of the neighbours. Return true if it was and
 * return false otherwise.
 *
 * TODO: Would it be better to return as soon as true is set?
 */
bool fromNeighbour() {
	bool result = false;

	if (neighbouringBoidCPUsSetup) {
		for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			if (inputData[CMD_FROM] == neighbouringBoidCPUs[i]) {
				result = true;
			}
		}
	}

	return result;
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void printCommand(bool send, uint32 *data) {
    if (send) {
        if (data[CMD_TO] == CONTROLLER_ID) {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to controller: ";
        } else if (data[CMD_TO] == BOIDGPU_ID) {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to BoidGPU: ";
        } else {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to " << data[CMD_TO] << ": ";
        }
    } else {
        if (data[CMD_FROM] == CONTROLLER_ID) {
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from controller: ";
        } else if (data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from BoidGPU: ";
        }  else {
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from " << data[CMD_FROM] << ": ";
        }
    }

    switch (data[CMD_TYPE]) {
        case 0:
            std::cout << "do something";
            break;
        case MODE_INIT:
            std::cout << "initialise self";
            break;
        case CMD_PING:
            std::cout << "BoidCPU ping";
            break;
        case CMD_PING_REPLY:
            std::cout << "BoidCPU ping response";
            break;
        case CMD_USER_INFO:
            std::cout << "output user info";
            break;
        case CMD_SIM_SETUP:
            std::cout << "setup BoidCPU";
            break;
        case MODE_CALC_NBRS:
            std::cout << "calculate neighbours";
            break;
        case CMD_NBR_REPLY:
            std::cout << "neighbouring boids from neighbour";
            break;
        case MODE_POS_BOIDS:
            std::cout << "calculate new boid positions";
            break;
        case MODE_LOAD_BAL:
			std::cout << "load balance";
			break;
		case CMD_LOAD_BAL:
			std::cout << "load balance instructions";
			break;
		case CMD_LOAD_BAL_REQUEST:
			std::cout << "load balance request";
			break;
		case CMD_BOUNDS_AT_MIN:
			std::cout << "BoidCPU at minimal bounds";
			break;
        case MODE_TRAN_BOIDS:
            std::cout << "transfer boids";
            break;
        case CMD_BOID:
            std::cout << "boid in transit";
            break;
        case MODE_DRAW:
            std::cout << "send boids to BoidGPU";
            break;
        case CMD_DRAW_INFO:
            std::cout << "boid info heading to BoidGPU";
            break;
        case CMD_ACK:
			std::cout << "ACK signal";
			break;
        case CMD_KILL:
            std::cout << "kill simulation";
            break;
        default:
            std::cout << "UNKNOWN COMMAND";
            break;
    }
    std::cout << std::endl;

    std::cout << "\t";
    printCommandLoop: for (int i = 0; i < CMD_HEADER_LEN; i++) {
        std::cout << data[i] << " ";
    }

    std::cout << "|| ";

    printCmdDataLoop: for (int i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
        std::cout << data[CMD_HEADER_LEN + i] << " ";
    }
    std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Classes /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//==============================================================================
// Boid ========================================================================
//==============================================================================

Boid::Boid() {
    id = 0;

    position = Vector(0, 0);
    velocity = Vector(0, 0);

    boidNeighbourIndex = 0;
    boidNeighbourCount = 0;
}

Boid::Boid(uint16 _boidID, Vector initPosition, Vector initVelocity) {
    id = _boidID;

    position = initPosition;
    velocity = initVelocity;

    boidNeighbourIndex = 0;
    boidNeighbourCount = 0;

    std::cout << "Created boid #" << id << std::endl;
    printBoidInfo();
}

void Boid::update(void) {
    std::cout << "Updating boid #" << id << std::endl;

    if (boidNeighbourCount > 0) {
        acceleration.add(separate());
        acceleration.add(align());
        acceleration.add(cohesion());
    }

    velocity.add(acceleration);
    velocity.limit(MAX_VELOCITY);
    position.add(velocity);
    acceleration.mul(0);

    printBoidInfo();
}

Vector Boid::align(void) {
    Vector total;

    alignBoidsLoop: for (int i = 0; i < boidNeighbourCount; i++) {
    	total.add(boidNeighbourList[boidNeighbourIndex][i]->velocity);
    }

    total.div(boidNeighbourCount);
    total.setMag(MAX_VELOCITY);

    Vector steer = Vector::sub(total, velocity);
    steer.limit(MAX_FORCE);

    return steer;
}

Vector Boid::separate(void) {
    Vector total;
    Vector diff;

    separateBoidsLoop: for (int i = 0; i < boidNeighbourCount; i++) {
        diff = Vector::sub(position, boidNeighbourList[boidNeighbourIndex][i]->position);
        diff.normalise();
        // Optionally weight by the distance
        total.add(diff);
    }

    total.div(boidNeighbourCount);
    total.setMag(MAX_VELOCITY);
    Vector steer = Vector::sub(total, velocity);
    steer.limit(MAX_FORCE);

    return steer;
}

Vector Boid::cohesion(void) {
    Vector total;

    coheseBoidLoop: for (int i = 0; i < boidNeighbourCount; i++) {
        total.add(boidNeighbourList[boidNeighbourIndex][i]->position);
    }

    total.div(boidNeighbourCount);

    Vector desired = Vector::sub(total, position);

    desired.setMag(MAX_VELOCITY);
	Vector steer = Vector::sub(desired, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

void Boid::setNeighbourDetails(uint8 neighbourIndex, uint8 neighbourCount) {
	boidNeighbourIndex = neighbourIndex;
	boidNeighbourCount = neighbourCount;
}

void Boid::printBoidInfo() {
    std::cout << "==========Info for Boid " << id << "==========" << std::endl;
    std::cout << "Boid Position: [" << position.x << " " << position.y << "]"
        << std::endl;
    std::cout << "Boid Velocity: [" << velocity.x << " " << velocity.y << "]"
        << std::endl;
    std::cout << "===================================" << std::endl;
}

//==============================================================================
// Vector ======================================================================
//==============================================================================

// Constructors ////////////////////////////////////////////////////////////////
Vector::Vector() {
    x = 0;
    y = 0;
}

Vector::Vector(int16_fp x_, int16_fp y_) {
    x = x_;
    y = y_;
}

// Basic Operations ////////////////////////////////////////////////////////////
void Vector::add(Vector v) {
    x = x + v.x;
    y = y + v.y;
}

void Vector::mul(int16_fp n) {
    x = x * n;
    y = y * n;
}

void Vector::div(int16_fp n) {
    if (n != 0) {
    	x = x / n;
    	y = y / n;
    }
}

// Static Operations /////////////////////////////////////////////////////////
Vector Vector::sub(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x - v2.x, v1.y - v2.y);
    return v3;
}

// Calculate the squared distance between two vectors - used to avoid use of
// doubles and square roots, which are expensive in hardware.
int16_fp Vector::squaredDistanceBetween(Vector v1, Vector v2) {
	int16_fp xPart = v1.x - v2.x;
	int16_fp yPart = v1.y - v2.y;

	xPart *= xPart;
	yPart *= yPart;

	return xPart + yPart;
}

// Advanced Operations /////////////////////////////////////////////////////////
int16_fp Vector::mag() {
	int16_fp result = (x*x + y*y);
	return hls::sqrt(result);
}

// TODO: setMag and limit are similar - perhaps combine?
void Vector::setMag(int16_fp mag) {
    normalise();
    mul(mag);
}

void Vector::limit(int16_fp max) {
	int16_fp m = mag();
    if (m > max) {
        normaliseWithMag(m);
        mul(max);
    }
}

void Vector::normalise() {
	normaliseWithMag(mag());
}

void Vector::normaliseWithMag(int16_fp magnitude) {
	if (magnitude != 0) {
		div(magnitude);
	} else {
		x = 0;
		y = 0;
	}
}
