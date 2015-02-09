#include "boidCPU.h"

#include <iostream>		// cout
#include <math.h>       // sqrt, floor

// Function headers ============================================================
// Key function headers --------------------------------------------------------
static void initialisation (void);
static void identify (void);
static void simulationSetup (void);
static void calcNextBoidPositions (void);
static void loadBalance (void);
static void moveBoids (void);
static void updateDisplay (void);

void sendBoidsToNeighbours(void);
void processNeighbouringBoids(void);

// Supporting function headers -------------------------------------------------
void transmitBoid(uint16 boidID, uint8 recipientID);
void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data);

int getRandom(int min, int max);
uint16 shiftLSFR(uint16 *lsfr, uint16 mask);

// Debugging function headers --------------------------------------------------
void printCommand(bool send, uint32 *data);
void printStateOfBoidCPUBoids();


// Global variables ============================================================
// TODO: Determine which variables should really be global
// BoidCPU variables ---------------------------------------------------------------
int8 boidCPUID;
int8 fpgaID;
int12 boidCPUCoords[4];

uint8 neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS];

uint32 inputData[MAX_CMD_LEN];
uint32 outputData[MAX_OUTPUT_CMDS][MAX_CMD_LEN];
uint32 outputBody[30];
uint32 outputCount = 0;				// The number of output messages buffered
bool outputAvailable = false;		// True if there is output ready to send

// Boid variables --------------------------------------------------------------
int8 boidCount;
Boid boids[MAX_BOIDS];		// TODO: Perhaps re-implement as a LL due to deletion

// TODO: Try with and without pointers (for synthesising)
Boid *boidNeighbourList[MAX_BOIDS][MAX_NEIGHBOURING_BOIDS];

// A list of possible neighbouring boids for the BoidCPU
Boid possibleNeighbouringBoids[MAX_NEIGHBOURING_BOIDS];
uint8 possibleNeighbourCount = 0;
// True when this BoidCPU has received a boid list from neighbouring BoidCPUs
bool receivedBoidCPUNbrList[MAX_BOIDCPU_NEIGHBOURS] = { false };

// Supporting variables --------------------------------------------------------
uint16 lfsr16 = 0xF429;	// LSFR seed - 16 bit binary (62505)
uint16 lfsr15 = 0x51D1;	// LSFR seed - 15 bit binary (20945)

// Debugging variables ---------------------------------------------------------
bool singleBoidCPU = false;
uint16 stopCondition;
uint16 timeStep = 0;

bool continueOperation = true;


/**
 * Fixed-point arithmetic attempt (failed)
 * --------------------------------------
 * Tried using HLS's fixed point library (ap_fixed.h) but it did not make sense.
 * For example:
 * 	ap_fixed<6,3, AP_RND, AP_WRAP> Val = 3.25;
 * 	std::cout << Val << std::endl; 				// Gives 3.25
 * However, this one didn't work for some reason:
 * 	din1_t a = -345.8;
 * 	std::cout << a << std::endl;				// Gives 0
 * 	fint12 b = -345.8;
 * 	std::cout << b << std::endl;				// Gives -346
 *
 * 	Aleks: <0, 1>, <0, 1>, <1, 2>, <2, 1>
 * 	Tom:   scale by 10000
 *
 * Where the types were defined in the header:
 * 	typedef ap_fixed<22,22, AP_RND, AP_SAT> fint12;
 * 	typedef ap_ufixed<10,8, AP_RND, AP_SAT> din1_t;
 */

/**
 * FIXME: When specifying stop at time step 100, it stops at time step 104...
 * TODO: Random generator needs random seeds - FPGA clock?
 * TODO: Programmatically calculate the length of the commands
 * TODO: Split data that is too long over multiple packets
 */

void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	// Perform initialisation
	// TODO: This only needs to be called on power up, not every time the
	//	BoidCPU is called.
	initialisation();

	// Continually check for input and deal with it. Note that reading an empty
	// input stream will generate warnings in HLS, but should be blocking in the
	// actual implementation.

	// TODO: Remove when deployment
	inputData[0] = input.read();

	while(continueOperation) {
		// INPUT ---------------------------------------------------------------
		// Block until there is input available
		// TODO: Remove comment when deployed
		//inputData[0] = input.read();

		// When there is input, read in the command
		inputLoop: for (int i = 0; i < inputData[CMD_LEN] - 1; i++) {
			inputData[1 + i] = input.read();
		}
		printCommand(false, inputData);
		// ---------------------------------------------------------------------

		// STATE CHANGE --------------------------------------------------------
		// TODO: Replace '83' with 'boidCPUID' when deploying
		if ((inputData[CMD_TO] == CMD_BROADCAST) || (inputData[CMD_TO] ==
				boidCPUID) || (inputData[CMD_TO] == 83)) {
			switch(inputData[CMD_TYPE]) {
				case MODE_INIT:
					initialisation();
					break;
				case CMD_PING:
					identify();
					break;
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
				case CMD_LOAD_BAL:
					loadBalance();
					break;
				case MODE_TRAN_BOIDS:
					moveBoids();
					break;
				case MODE_DRAW:
					updateDisplay();
					break;
				default:
					std::cout << "Command state " << inputData[CMD_TYPE] <<
						" not recognised" << std::endl;
					break;
			}
		}
		// ---------------------------------------------------------------------

		// OUTPUT --------------------------------------------------------------
		// If there is output to send, send it
		if (outputAvailable) {
			outerOutputLoop: for (int j = 0; j < outputCount; j++) {
				innerOutputLoop: for (int i = 0; i < outputData[j][CMD_LEN]; i++) {
					output.write(outputData[j][i]);
				}
				printCommand(true, outputData[j]);
			}
		}
		outputAvailable = false;
		outputCount = 0;
		// ---------------------------------------------------------------------

		// TODO: Remove when deployed
		continueOperation = input.read_nb(inputData[0]);
	}
	std::cout << "======BoidCPU has finished======" << std::endl;
}

//==============================================================================
// State functions =============================================================
//==============================================================================

/**
 * TODO: Setup necessary components - maybe?
 * Generate a random initial ID
 * TODO: Identify the serial number of the host FPGA
 */
void initialisation() {
	std::cout << "-Initialising BoidCPU..." << std::endl;

	boidCPUID = getRandom(1, 100);
	fpgaID = 123;

	std::cout << "-Waiting for ping from Boid Controller..." << std::endl;
}

/**
 * Waits for a ping from the controller
 * Transmits its temporary ID and FPGA serial number to the controller
 */
void identify() {
	outputBody[0] = boidCPUID;
	outputBody[1] = fpgaID;
	generateOutput(2, CONTROLLER_ID, CMD_PING_REPLY, outputBody);
	// 6, 1, [RANDOM ID], 3 || [RANDOM ID], 123

	std::cout << "-Responded to ping" << std::endl;
}

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
	boidCPUID = inputData[CMD_HEADER_LEN + 0];
	boidCount = inputData[CMD_HEADER_LEN + 1];

	edgeSetupLoop: for (int i = 0; i < EDGE_COUNT; i++) {
		boidCPUCoords[i] = inputData[CMD_HEADER_LEN + 2 + i];
	}

	neighbourSetupLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		neighbouringBoidCPUs[i] = inputData[CMD_HEADER_LEN + EDGE_COUNT + 2 + i];
	}

	// If the value is not 0 then the BoidCPU is able to progress itself until
	// it reaches the time step equal to the supplied value - it does not need
	// to wait for the controller to supply synchronisation steps
//	if (inputData[18] > 0) {
////		singleBoidCPU = true;
//		stopCondition = inputData[18];
//	}

	// Print out BoidCPU parameters
	std::cout << "BoidCPU #" << oldBoidCPUID << " now has ID #" << boidCPUID << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " initial boid count: " << boidCount << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " coordinates: [";
	printEdgeLoop: for (int i = 0; i < EDGE_COUNT; i++) {
		std::cout << boidCPUCoords[i] << ", ";
	} std::cout << "]" << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " neighbours: [";
	printNeighbourLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		std::cout << neighbouringBoidCPUs[i] << ", ";
	} std::cout << "]" << std::endl;

	// Create the boids (actual implementation)
//	uint16 boidID;
//	boidCreationLoop: for(int i = 0; i < boidCount; i++) {
//		Vector velocity = Vector(getRandom(-MAX_VELOCITY, MAX_VELOCITY), getRandom(-MAX_VELOCITY, MAX_VELOCITY));
//		Vector position = Vector(getRandom(boidCPUCoords[X_MIN], boidCPUCoords[X_MAX]), getRandom(boidCPUCoords[Y_MIN], boidCPUCoords[Y_MAX]));
//
//		boidID = ((boidCPUID - 1) * boidCount) + i + 1;
//
//		Boid boid = Boid(boidID, position, velocity, i);
//		boids[i] = boid;
//	}

	// Create the boids (testing implementation)
	int testBoidCount = 10;
	Vector knownSetup[10][2] = {{Vector(12, 11), Vector(5, 0)},
			{Vector(19, 35), Vector(-5, 1)},
			{Vector(12, 31), Vector(-4, -2)},
			{Vector(35, 22), Vector(0, -3)},
			{Vector(4, 9), Vector(-1, 0)},
			{Vector(19, 18), Vector(2, -3)},
			{Vector(38, 19), Vector(4, -4)},
			{Vector(18, 5), Vector(-1, 2)},
			{Vector(15, 33), Vector(2, -2)},
			{Vector(3, 8), Vector(-2, 0)}};

	testBoidCreationLoop: for(int i = 0; i < testBoidCount; i++) {
		uint16 boidID = ((boidCPUID - 1) * boidCount) + i + 1;
		Boid boid = Boid(boidID, knownSetup[i][0], knownSetup[i][1], i);
		boids[i] = boid;
	}
}

/**
 * Send this BoidCPU's boids to its neighbouring BoidCPUs so they can use them
 * in calculating neighbours for their boids. Splits the data to be sent into
 * multiple messages if it exceeds the maximum command data size.
 */
void sendBoidsToNeighbours() {
	std::cout << "-Sending boids to neighbouring BoidCPUs..." << std::endl;

	// First, calculate how many messages need to be sent
	// Division by subtraction needed due to issues with integer division
	uint16 numerator = boidCount * BOID_DATA_LENGTH;
	uint16 msgCount = 0;
	for(msgCount = 0; numerator > 0; msgCount++){
		numerator -=MAX_CMD_BODY_LEN;
	}

	// Then calculate the number of boids that can be sent per message
	uint16 boidsPerMsg = MAX_CMD_BODY_LEN / BOID_DATA_LENGTH;

	// Next, send a message for each group of boids
	for (int i = 0; i < msgCount; i++) {
		// Determine the boid indexes for this message
		int startBoidIndex 	= i * boidsPerMsg;
		int endBoidIndex 	= startBoidIndex + boidsPerMsg;

		// Limit the end index if the message won't be full
		if (endBoidIndex > boidCount) {
			endBoidIndex = boidCount;
		}

		// The next step is to create the message data
		for (int j = startBoidIndex, k = 0; j < endBoidIndex; j++, k++) {
			uint32 position = 0;
			uint32 velocity = 0;

			position |= ((uint32)(boids[j].position.x) << 20);
			position |= ((uint32)(boids[j].position.y) << 8);

			// Despite being of type int12, the velocity (and position) seem to
			// be represented using 16 bits. Therefore, negative values need to
			// have bits 12 to 15 set to 0 (from 1) before ORing with velocity.
			if(boids[j].velocity.x < 0) {
				velocity |= ((uint32)((boids[j].velocity.x) & ~((int16)0x0F << 12)) << 20);
			} else {
				velocity |= ((uint32)(boids[j].velocity.x) << 20);
			}

			if(boids[j].velocity.y < 0) {
				velocity |= ((uint32)((boids[j].velocity.y) & ~((int16)0x0F << 12)) << 8);
			} else {
				velocity |= ((uint32)(boids[j].velocity.y) << 8);
			}

			outputBody[(k * BOID_DATA_LENGTH) + 0] = position;
			outputBody[(k * BOID_DATA_LENGTH) + 1] = velocity;
			outputBody[(k * BOID_DATA_LENGTH) + 2] = boids[j].id;
		}

		// Finally send the message to each neighbour
		for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
			int dataLength = ((endBoidIndex - startBoidIndex)) * BOID_DATA_LENGTH;
			generateOutput(dataLength, neighbouringBoidCPUs[i], CMD_NBR_REPLY, outputBody);
		}
	}

	// Now, add the BoidCPU's own boids to the possible neighbouring boids list
	// The boids from the current BoidCPU need to be first
	addOwnAsNbrsLoops: for (int i = 0; i < boidCount; i++) {
		possibleNeighbouringBoids[i] = boids[i];
		possibleNeighbourCount++;
	}
}

/**
 * When boids are received from neighbouring BoidCPUs, process them and add
 * them to a list of possible neighbouring boids. These are used to calculate
 * the neighbours of boids contained within this BoidCPU.
 */
void processNeighbouringBoids() {
	// First, get the number of boids that the message contains
	int count = (inputData[CMD_LEN] - CMD_HEADER_LEN) / BOID_DATA_LENGTH;

	std::cout << "-BoidCPU #" << boidCPUID << " received " << count <<
		" boids from BoidCPU #" << inputData[CMD_FROM] << std::endl;

	// Then, create a boid object for each listed boid and add to the list of
	// possible neighbouring boids for this BoidCPU
	for (int i = 0; i < count; i++) {
		uint32 position = inputData[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 0];
		uint32 velocity = inputData[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 1];

		Vector p = Vector((int12)((position & (~(uint32)0xFFFFF)) >> 20),
				(int12)((position & (uint32)0xFFF00) >> 8));

		Vector v = Vector((int12)((velocity & (~(uint32)0xFFFFF)) >> 20),
				(int12)((velocity & (uint32)0xFFF00) >> 8));

		Boid b = Boid((uint16)inputData[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 2], p, v, i);
		possibleNeighbouringBoids[possibleNeighbourCount] = b;
		possibleNeighbourCount++;
	}

	// Then mark the neighbouring BoidCPU as having sent its boids
	receivedBoidCPUNbrList[inputData[CMD_FROM]] = true;

	// Set a flag if all the neighbouring BoidCPUs have sent their boids
	// TODO: Could use a counter instead
	bool allNeighboursReceived = false;
	for (int i = 0; i < MAX_NEIGHBOURING_BOIDS; i++) {
		allNeighboursReceived = (allNeighboursReceived && receivedBoidCPUNbrList[i]);
	}

	// If the flag is true, calculate the neighbours for all the boids
	// TODO: Try moving the variables outside the loop to see what happens
	//
	// 'if (i != j)' relies on the possible neighbour list starting with the
	// boids of the current BoidCPU
	if (allNeighboursReceived) {
		boidCPUCalcBoidNbrsLoop: for (int i = 0; i < boidCount; i++) {
			int neighbouringBoidsCount = 0;
			uint16 distance;

			calcBoidNbrsLoop: for (int j = 0; j < possibleNeighbourCount; j++) {
				if (i != j) {
//				if (possibleNeighbouringBoids[j].id != boids[i].id) {
					distance = Vector::distanceBetween(boids[i].position,
						possibleNeighbouringBoids[j].position);

					if (distance < VISION_RADIUS) {
						boidNeighbourList[i][neighbouringBoidsCount] =
							&possibleNeighbouringBoids[j];
						neighbouringBoidsCount++;
					}
				}
			}

			boids[i].setNeighbourCount(neighbouringBoidsCount);
		}
	}
}

void calcNextBoidPositions() {
	std::cout << "-Calculating next boid positions..." << std::endl;

	updateBoidsLoop: for (int i = 0; i < boidCount; i++) {
		boids[i].update();
	}

	// If testing only a single BoidCPU, continue to next stage
//	if (singleBoidCPU) {
//		moveBoids();
//	}
}

void loadBalance() {
	std::cout << "-Load balancing..." << std::endl;
}

void moveBoids() {
	std::cout << "-Transferring boids..." << std::endl;

	uint16 idOfBoidToTransfer;
//	Boid boidToTransfer;
	uint8 recipientBoidCPU;

	moveBoidsLoop: for (int i = 0; i < boidCount; i++) {
		recipientBoidCPU = 0;

		if ((neighbouringBoidCPUs[0] != 0) && (boids[i].position.y < boidCPUCoords[1]) && (boids[i].position.x < boidCPUCoords[0])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[0];
		} else if ((neighbouringBoidCPUs[2] != 0) && (boids[i].position.y < boidCPUCoords[1]) && (boids[i].position.x > boidCPUCoords[2])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[2];
		} else if ((neighbouringBoidCPUs[4] != 0) && (boids[i].position.y > boidCPUCoords[3]) && (boids[i].position.x > boidCPUCoords[2])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[4];
		} else if ((neighbouringBoidCPUs[6] != 0) && (boids[i].position.y > boidCPUCoords[3]) && (boids[i].position.x < boidCPUCoords[0])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[6];
		} else if ((neighbouringBoidCPUs[1] != 0) && (boids[i].position.y < boidCPUCoords[1])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[1];
		} else if ((neighbouringBoidCPUs[3] != 0) && (boids[i].position.x > boidCPUCoords[2])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[3];
		} else if ((neighbouringBoidCPUs[5] != 0) && (boids[i].position.y > boidCPUCoords[3])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[5];
		} else if ((neighbouringBoidCPUs[7] != 0) && (boids[i].position.x < boidCPUCoords[0])) {
			idOfBoidToTransfer = i;
			recipientBoidCPU = neighbouringBoidCPUs[7];
		}

		if (recipientBoidCPU > 0) {
			transmitBoid(idOfBoidToTransfer, recipientBoidCPU);
		}
	}

	// If testing only a single BoidCPU, continue to next stage
//	if (singleBoidCPU) {
//		updateDisplay();
//	}
}

void updateDisplay() {
	std::cout << "-Updating display" << std::endl;

	// TODO: On deployment, don't need to send ID, but need to know the
	//	direction of the boid so either send full velocity or angle
	// TODO: Decide on breakdown, should boids be sent all in one message, all
	//	in separate messages or a mixture of the two?
	for (int i = 0; i < boidCount; i++) {
		outputBody[(3 * i) + 0] = boids[i].id;
		outputBody[(3 * i) + 1] = boids[i].position.x;
		outputBody[(3 * i) + 2] = boids[i].position.y;
	}

	generateOutput((3 * boidCount), BOIDGPU_ID, CMD_DRAW_INFO, outputBody);
	// [4 + (3 * boidCount)], 2, 6, 13 || [boid information]

	// Increment the time step counter
	timeStep++;

	printStateOfBoidCPUBoids();

//	// If testing only a single BoidCPU, continue to next stage
//	if (singleBoidCPU) {
//		// If the current time step is less than the stopping condition, continue
////		if (boidCount > 0)
//		if (timeStep < stopCondition) {
//			findNeighbours();
//		}
//	}
}

void printStateOfBoidCPUBoids() {
	for (int i = 0; i < boidCount; i++) {
		std::cout << "Boid " << boids[i].id << " has position [" <<
			boids[i].position.x << ", " << boids[i].position.y <<
			"] and velocity [" << boids[i].velocity.x << ", " <<
			boids[i].velocity.y << "]" << std::endl;
	}
}

//==============================================================================
// Supporting functions ========================================================
//==============================================================================

// Generic methods
void transmit(int to, int data) {
//	output.write('a');
}

void receive() {
//	input.read();

	// Identify the input and if it is a boid, send data to function
//	acceptBoid(inputData);
}

// Take a copy of the boid, not the actual one
void transmitBoid(uint16 boidID, uint8 recipientID) {
	// TODO: Perhaps move this to the boid class?
	outputBody[0] = boids[boidID].id;
	outputBody[1] = boids[boidID].position.x;
	outputBody[2] = boids[boidID].position.y;
	outputBody[3] = boids[boidID].velocity.x;
	outputBody[4] = boids[boidID].velocity.y;

	generateOutput(7, recipientID, CMD_BOID, outputBody);

	// Remove boid from own list
	bool boidFound = false;
	for (int i = 0; i < boidCount - 1; i++) {
		if (i == boidID) {
			boidFound = true;
		}

		if (boidFound) {
			boids[i] = boids[i + 1];
		}
	}
	boidCount--;

	std::cout << "-Transferring boid #" << boids[boidID].id << " to boidCPU #"
		<< recipientID << std::endl;
}

void acceptBoid(uint32 *boidData) {
	// TODO: Parse the input data to create a boid object
	int boidID = 12;
	Vector boidPosition = Vector(12, 100);
	Vector boidVelocity = Vector(10, -2);
	Boid b = Boid(boidID, boidPosition, boidVelocity, boidCount);

	boids[boidCount] = b;
	boidCount++;
}

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data) {
	if (outputCount > MAX_OUTPUT_CMDS - 1) {

		std::cout << "Cannot send message, output buffer is full (" <<
			outputCount << ")" << std::endl;
	} else {
		outputData[outputCount][CMD_LEN]  = len + CMD_HEADER_LEN;
		outputData[outputCount][CMD_TO]   = to;
		outputData[outputCount][CMD_FROM] = boidCPUID;
		outputData[outputCount][CMD_TYPE] = type;

		if (len > 0) {
			createOutputCommandLoop: for(int i = 0; i < len; i++) {
				outputData[outputCount][CMD_HEADER_LEN + i] = data[i];
			}
		}
		outputCount++;
		outputAvailable = true;
	}
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void printCommand(bool send, uint32 *data) {
	if(send) {
		if(data[CMD_TO] == CONTROLLER_ID) {
			std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to controller: ";
		} else if(data[CMD_TO] == BOIDGPU_ID) {
			std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to BoidGPU: ";
		} else {
			std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to " << data[CMD_TO] << ": ";
		}
	} else {
		if(data[CMD_FROM] == CONTROLLER_ID) {
			std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from controller: ";
		} else if(data[CMD_FROM] == BOIDGPU_ID) {
			// This should never happen
			std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from BoidGPU: ";
		}  else {
			std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from " << data[CMD_FROM] << ": ";
		}
	}

	switch(data[CMD_TYPE]) {
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
		case CMD_LOAD_BAL:
			std::cout << "load balance";
			break;
		case MODE_TRAN_BOIDS:
			std::cout << "transfer boids";
			break;
		case CMD_BOID:
			std::cout << "boid";
			break;
		case MODE_DRAW:
			std::cout << "send boids to BoidGPU";
			break;
		case CMD_DRAW_INFO:
			std::cout << "boid info heading to BoidGPU";
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
	printCommandLoop: for(int i = 0; i < CMD_HEADER_LEN; i++) {
		std::cout << data[i] << " ";
	}

	std::cout << "|| ";

	printCommandDataLoop: for(int i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
		std::cout << data[CMD_HEADER_LEN + i] << " ";
	}
	std::cout << std::endl;
}

//==============================================================================
// Random ======================================================================
//==============================================================================

// TODO: Try with 32 bits
// http://en.wikipedia.org/wiki/Linear_feedback_shift_register#Galois_LFSRs
// http://stackoverflow.com/q/17764587
// http://stackoverflow.com/a/5009006
int getRandom(int min, int max) {
	shiftLSFR(&lfsr16, POLY_MASK_16);
	int result = (shiftLSFR(&lfsr16, POLY_MASK_16) ^ shiftLSFR(&lfsr15, POLY_MASK_15));

	return (min + (result % (int)(max - min + 1)));
}

uint16 shiftLSFR(uint16 *lfsr, uint16 mask) {
	uint16 lsb = *lfsr & 1;		// Get LSB (i.e., the output bit)
	*lfsr >>= 1;				// Shift register
	if (lsb == 1)				// Only apply toggle mask if output bit is 1
		*lfsr ^= 0xB400u;		// Apply toggle mask, value has 1 at bits
								//  corresponding to taps, 0 elsewhere
	return *lfsr;
}

////////////////////////////////////////////////////////////////////////////////
// Classes /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//==============================================================================
// Boid ========================================================================
//==============================================================================

Boid::Boid() {
	id = 0;
	index = 0;

	position = Vector(0, 0);
	velocity = Vector(0, 0);

	neighbouringBoidsCount = 0;
}

Boid::Boid(uint16 _boidID, Vector initPosition, Vector initVelocity, int _index) {
	id = _boidID;
	index = _index;

	position = initPosition;
	velocity = initVelocity;

	neighbouringBoidsCount = 0;

	std::cout << "Created boid #" << id << std::endl;
//	printBoidInfo();
}

void Boid::update(void) {
	std::cout << "Updating boid #" << id << std::endl;

	if(neighbouringBoidsCount > 0) {
		acceleration.add(separate());
		acceleration.add(align());
		acceleration.add(cohesion());
	}

	velocity.add(acceleration);
	velocity.limit(MAX_VELOCITY);
	position.add(velocity);
	acceleration.mul(0);

	contain();
}

Vector Boid::align(void) {
	Vector total;

	alignBoidsLoop: for (int i = 0; i < neighbouringBoidsCount; i++) {
		total.add(boidNeighbourList[index][i]->velocity);
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::separate(void) {
	Vector total;
	Vector diff;

	separateBoidsLoop: for (int i = 0; i < neighbouringBoidsCount; i++) {
		diff = Vector::sub(position, boidNeighbourList[index][i]->position);
		diff.normalise();
		total.add(diff);
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);
	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::cohesion(void) {
	Vector total;
	Vector steer;

	coheseBoidLoop: for (int i = 0; i < neighbouringBoidsCount; i++) {
		total.add(boidNeighbourList[index][i]->position);
	}

	total.div(neighbouringBoidsCount);
	steer = seek(total);

	return steer;
}

Vector Boid::seek(Vector target) {
	Vector desired = Vector::sub(target, position);
	desired.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(desired, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

void Boid::contain() {
	if(position.x > AREA_WIDTH) {
		position.x = 0;
	} else if(position.x < 0) {
		position.x = AREA_WIDTH;
	}

	if(position.y > AREA_HEIGHT) {
		position.y = 0;
	} else if(position.y < 0) {
		position.y = AREA_HEIGHT;
	}
}

void Boid::setNeighbourCount(int n) {
	neighbouringBoidsCount = (uint8)n;
}

void Boid::printBoidInfo() {
	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
	std::cout << "Boid Position: [" << position.x << " " << position.y << "]" << std::endl;
	std::cout << "Boid Velocity: [" << velocity.x << " " << velocity.y << "]" << std::endl;
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

Vector::Vector(int12 x_, int12 y_) {
	x = x_;
	y = y_;
}

// Basic Operations ////////////////////////////////////////////////////////////
void Vector::add(Vector v) {
	x = x + v.x;
	y = y + v.y;
}

void Vector::sub(Vector v) {
	x = x - v.x;
	y = y - v.y;
}

void Vector::mul(int12 n) {
	x = x * n;
	y = y * n;
}

void Vector::div(int12 n) {
	if (n != 0) {
		x = x / n;
		y = y / n;
	}
}

// Static Operations /////////////////////////////////////////////////////////
Vector Vector::add(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x + v2.x, v1.y + v2.y);
	return v3;
}

Vector Vector::sub(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x - v2.x, v1.y - v2.y);
	return v3;
}

// FIXME: The sqrt takes a double, probably will be expensive in h/w
double Vector::distanceBetween(Vector v1, Vector v2) {
	int12 xPart = v1.x - v2.x;
	int12 yPart = v1.y - v2.y;

	xPart = xPart * xPart;
	yPart = yPart * yPart;

	// Could also use hls::sqrt() - in newer HLS version
	return sqrt(double(xPart + yPart));
}

bool Vector::equal(Vector v1, Vector v2) {
	if ((v1.x == v2.x) && (v1.y == v2.y)) {
		return true;
	} else {
		return false;
	}
}

// Advanced Operations /////////////////////////////////////////////////////////
int12 Vector::mag() {
	// Could also use hls::sqrt() - in newer HLS version
	return (uint12)round(sqrt(double(x*x + y*y)));
}

void Vector::setMag(int12 mag) {
	normalise();
	mul(mag);
}

void Vector::normalise() {
	int12 m = mag();

	if (m != 0) {
		div(m);
	} else {
		x = 0;
		y = 0;
	}
}

void Vector::limit(int12 max) {
	if(mag() > max) {
		normalise();
		mul(max);
	}
}

void Vector::bound(int12 n) {
	// TODO: The is technically not binding the speed, which is the magnitude
	if(x > n) x = n;
	if(y > n) y = n;
}

bool Vector::empty() {
	bool result = true;

	if(x) result = false;
	else if(y) result = false;

	return result;
}
