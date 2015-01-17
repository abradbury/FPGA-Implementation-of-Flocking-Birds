#include "boidCPU.h"

#include <iostream>		// cout
#include <math.h>       // sqrt

#define MAX_BOIDS				30
#define MAX_VELOCITY			10
#define MAX_FORCE				1
#define MAX_CMD_LEN				10		// TODO: Decide on appropriate value

#define AREA_WIDTH				720		// TODO: Should a BoidCPU know this?
#define AREA_HEIGHT				720		// TODO: Should a BoidCPU know this?
#define EDGE_COUNT				4		// The number of edges a BoidCPU has

#define CMD_HEADER_LEN			4

#define VISION_RADIUS				100
#define MAX_NEIGHBOURING_BOIDCPUS	8		// The max neighbours a BoidCPUs has
#define MAX_NEIGHBOURING_BOIDS		90		// TODO: Decide on appropriate value?

#define POLY_MASK_16			0xD295
#define POLY_MASK_15			0x6699

#define X_MIN			0	// The coordinate index of the minimum x position
#define Y_MIN			1	// The coordinate index of the minimum y position
#define X_MAX			2	// The coordinate index of the maximum x position
#define Y_MAX			3	// The coordinate index of the maximum y position

#define CMD_LEN			0	// The index of the command length
#define CMD_TO			1	// The index of the command target
#define CMD_FROM		2	// The index of the command sender
#define	CMD_TYPE		3	// The index of the command type

#define CMD_BROADCAST	0	// The number representing a broadcast command

#define MODE_INIT 		1	//
#define	CMD_PING		2	// Controller -> BoidCPU
#define CMD_PING_REPLY	3	// BoidCPU -> Controller
#define CMD_USER_INFO	4	// Controller -> BoidGPU
#define CMD_SIM_SETUP	5	// Controller -> Boid[CG]PU
#define MODE_CALC_NBRS	6	//
#define CMD_NBR_REQUEST	7	// BoidCPU -> BoidCPU
#define CMD_NBR_REPLY	8	// BoidCPU -> BoidCPU
#define MODE_POS_BOIDS	9	//
#define CMD_LOAD_BAL	10	// TODO: Decide on implementation
#define MODE_TRAN_BOIDS	11	//
#define MODE_DRAW		12	// TODO: Perhaps not needed?
#define CMD_DRAW_INFO	13	// BoidCPU -> BoidGPU
#define CMD_KILL		14	// Controller -> All

// Function headers
static void initialisation (void);
static void identify (void);
static void simulationSetup (void);
static void findNeighbours (void);
static void calcNextBoidPositions (void);
static void loadBalance (void);
static void moveBoids (void);
static void updateDisplay (void);

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data);
void printCommand(bool send, uint32 *data);

int getRandom(int min, int max);
int shiftLSFR(uint16 *lsfr, uint16 mask);

// Define variables
// TODO: Should these really be global?
int8 boidCPUID;
int8 fpgaID;

int8 boidCount;
Boid boids[MAX_BOIDS];
uint8 neighbouringBoidCPUs[MAX_NEIGHBOURING_BOIDCPUS];
int12 boidCPUCoords[4];

uint32 inputData[MAX_CMD_LEN];
uint32 outputData[MAX_CMD_LEN];
uint32 outputBody[20];

// LSFR seed values
uint16 lfsr16 = 0xF429;	// 16 bit binary (62505)
uint16 lfsr15 = 0x51D1;	// 15 bit binary (20945)

bool outputAvailable = false;		// True if there is output ready to send

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
 * Where the types were defined in the header:
 * 	typedef ap_fixed<22,22, AP_RND, AP_SAT> fint12;
 * 	typedef ap_ufixed<10,8, AP_RND, AP_SAT> din1_t;
 */

Boid *neighbouringBoids[MAX_NEIGHBOURS];

/**
 * TODO: Sync states and command types?
 * TODO: Argument 'this' of function 'getVelocity' has an unsynthesizable type?
 * TODO: Random generator needs random seeds - FPGA clock?
 * TODO: Try and programmatically calculate the length of the commands
 */

void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	// Perform initialisation
	// TODO: This only needs to be called on power up, not every time the
	//	BoidCPU is called.
	initialisation();

	// INPUT -------------------------------------------------------------------
	// Block until there is input available
	inputData[0] = input.read();

	// When there is input, read in the command
	inputLoop: for (int i = 0; i < inputData[CMD_LEN] - 1; i++) {
		inputData[1 + i] = input.read();
	}
	printCommand(false, inputData);
	// -------------------------------------------------------------------------

	// STATE CHANGE ------------------------------------------------------------
	// TODO: Replace '6' with 'boidCPUID' when deploying
	if ((inputData[CMD_TO] == CMD_BROADCAST) || (inputData[CMD_TO] == 6)) {
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
				findNeighbours();
				break;
			case CMD_NBR_REQUEST:
//				sendBoidsToNeighbour();
				break;
			case CMD_NBR_REPLY:
//				processNeighbouringBoids();
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
	// -------------------------------------------------------------------------

	// OUTPUT ------------------------------------------------------------------
	// If there is output to send, send it
	if (outputAvailable) {
		outputLoop: for (int i = 0; i < outputData[CMD_LEN]; i++) {
			output.write(outputData[i]);
		}
		printCommand(true, outputData);
	}
	// -------------------------------------------------------------------------
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
	generateOutput(2, 0, CMD_PING_REPLY, outputBody);
	// 6, 0, [RANDOM ID], 3 || [RANDOM ID], 123

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

	neighbourSetupLoop: for (int i = 0; i < MAX_NEIGHBOURING_BOIDCPUS; i++) {
		neighbouringBoidCPUs[i] = inputData[CMD_HEADER_LEN + EDGE_COUNT + 2 + i];
	}

	// Print out BoidCPU parameters
	std::cout << "BoidCPU #" << oldBoidCPUID << " now has ID #" << boidCPUID << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " initial boid count: " << boidCount << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " coordinates: [";
	printEdgeLoop: for (int i = 0; i < EDGE_COUNT; i++) {
		std::cout << boidCPUCoords[i] << ", ";
	} std::cout << "]" << std::endl;
	std::cout << "BoidCPU #" << boidCPUID << " neighbours: [";
	printNeighbourLoop: for (int i = 0; i < MAX_NEIGHBOURING_BOIDCPUS; i++) {
		std::cout << neighbouringBoidCPUs[i] << ", ";
	} std::cout << "]" << std::endl;

	// Create the boids (actual implementation)
	boidCreationLoop: for(int i = 0; i < boidCount; i++) {
		Vector position = Vector(getRandom(-MAX_VELOCITY, MAX_VELOCITY), getRandom(-MAX_VELOCITY, MAX_VELOCITY), 0);
		Vector velocity = Vector(getRandom(boidCPUCoords[X_MIN], boidCPUCoords[X_MAX]), getRandom(boidCPUCoords[Y_MIN], boidCPUCoords[Y_MAX]), 0);

		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;

		Boid boid = Boid(boidID, position, velocity);
		boids[i] = boid;
	}

	// Create the boids (testing implementation)
//	int testBoidCount = 10;
//	Vector knownSetup[10][2] = {{Vector(695, 252, 0), Vector(-5, -9, 0)},
//			{Vector(594, 404, 0), Vector(-10, -1, 0)},
//			{Vector(550, 350, 0), Vector(-10, -3, 0)},
//			{Vector(661, 446, 0), Vector(-6, -4, 0)},
//			{Vector(539, 283, 0), Vector(-8, -2, 0)},
//			{Vector(551, 256, 0), Vector(-5, 7, 0)},
//			{Vector(644, 342, 0), Vector(-1, -7, 0)},
//			{Vector(592, 399, 0), Vector(-9, 6, 0)},
//			{Vector(644, 252, 0), Vector(-5, -8, 0)},
//			{Vector(687, 478, 0), Vector(-9, 9, 0)}};
//
//	testBoidCreationLoop: for(int i = 0; i < testBoidCount; i++) {
//		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;
//		Boid boid = Boid(boidID, knownSetup[i][0], knownSetup[i][1]);
//		boids[i] = boid;
//	}
}

void findNeighbours() {
	std::cout << "-Finding neighbouring boids..." << std::endl;

	//--------------------------------------------------------------------------
	// TODO: Remove this when deployed
	boidCount = 10;
	boidCPUID = 6;
	Vector knownSetup[10][2] = {{Vector(695, 252, 0), Vector(-5, -9, 0)},
			{Vector(594, 404, 0), Vector(-10, -1, 0)},
			{Vector(550, 350, 0), Vector(-10, -3, 0)},
			{Vector(661, 446, 0), Vector(-6, -4, 0)},
			{Vector(539, 283, 0), Vector(-8, -2, 0)},
			{Vector(551, 256, 0), Vector(-5, 7, 0)},
			{Vector(644, 342, 0), Vector(-1, -7, 0)},
			{Vector(592, 399, 0), Vector(-9, 6, 0)},
			{Vector(644, 252, 0), Vector(-5, -8, 0)},
			{Vector(687, 478, 0), Vector(-9, 9, 0)}};

	tmpBoidCreationLoop: for(int i = 0; i < boidCount; i++) {
		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;
		Boid boid = Boid(boidID, knownSetup[i][0], knownSetup[i][1]);
		boids[i] = boid;
	}
	//--------------------------------------------------------------------------

	// Generate a list of possible neighbouring boids
	Boid possibleNeighbouringBoids[MAX_NEIGHBOURING_BOIDS];
	uint8 possibleNeighbourCount = 0;

	// Add the boids of the current BoidCPU to the possible neighbour list
	addOwnAsNbrsLoops: for (int i = 0; i < boidCount; i++) {
		possibleNeighbouringBoids[i] = boids[i];
		possibleNeighbourCount++;
	}

	// Add the boids from the neighbouring BoidCPUs to the possible neighbour list
	getOthersAsNbrsLoop: for (int i = boidCount; i < MAX_NEIGHBOURING_BOIDCPUS; i++) {
//		if (neighbouringBoidCPUs[i] != 0) {
			// TODO: Get boids from neighbouring BoidCPUs
			// possibleNeighbouringBoids[] =
			// possibleNeighbourCount++;
//		}
	}

	// Calculate the neighbours for each boid
	// TODO: This could be done when calculating each boid's positions as C/C++
	//	seems to be pass by value - unlike Python
	calcBoidNbrsLoop: for (int i = 0; i < boidCount; i++) {
		boids[i].calculateNeighbours(possibleNeighbouringBoids, possibleNeighbourCount);
	}

//	// A list of the known neighbours for the boids at time step 1 (from Python)
//	// Sat 10th Jan: HLS is correct
//	int knownSetupNeighbours [10][5] = {{59},
//			 {53, 54, 57, 58},
//			 {52, 55, 56, 57, 58},
//			 {52, 58, 60},
//			 {53, 56},
//			 {53, 55, 59},
//			 {52, 53, 58, 59},
//			 {52, 53, 54, 57},
//			 {51, 56, 57},
//			 {54}};
}

void calcNextBoidPositions() {
	std::cout << "-Calculating next boid positions..." << std::endl;

	updateBoidsLoop: for (int i = 0; i < boidCount; i++) {
		boids[i].update();
	}

//	// The next positions of the boids, rounded to ints from floats, from Python
//	Vector knownSetupNextPos[10][2] = {{Vector(691, 244, 0), Vector(-4, -8, 0)},
//			{Vector(586, 403, 0), Vector(-8, -2, 0)},
//			{Vector(541, 349, 0), Vector(-9, -2, 0)},
//			{Vector(655, 444, 0), Vector(-6, -2, 0)},
//			{Vector(532, 283, 0), Vector(-7, -2, 0)},
//			{Vector(547, 261, 0), Vector(-4, 5, 0)},
//			{Vector(642, 337, 0), Vector(-2, -5, 0)},
//			{Vector(584, 403, 0), Vector(-8, 4, 0)},
//			{Vector(639, 245, 0), Vector(-5, -7, 0)},
//			{Vector(680, 485, 0), Vector(-7, 7, 0)}};
//
//	for (int i = 0; i < boidCount; i++) {
//		if (!Vector::equal(boids[i].position, knownSetupNextPos[i][0])) {
//			std::cout << "Boid #" << boids[i].id << " position differs" << std::endl;
//			std::cout << "   " << boids[i].position << " vs " << knownSetupNextPos[i][0] << std::endl;
//		} else {
//			std::cout << "Boid #" << boids[i].id << " position same" << std::endl;
//		}
//	}
}

void loadBalance() {
	std::cout << "-Load balancing..." << std::endl;
}

void moveBoids() {
	std::cout << "-Transferring boids..." << std::endl;

	Boid boidToTransfer;
	int recipientBoidCPU = 0;

	moveBoidsLoop: for (int i = 0; i < boidCount; i++) {
		if (neighbouringBoidCPUs[0] != 0) {
			if ((boids[i].position.y < boidCPUCoords[1]) && (boids[i].position.x < boidCPUCoords[0])) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[0];
			}
		} else if (neighbouringBoidCPUs[2] != 0) {
			if ((boids[i].position.y < boidCPUCoords[1]) && (boids[i].position.x > boidCPUCoords[2])) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[2];
			}
		} else if (neighbouringBoidCPUs[4] != 0) {
			if ((boids[i].position.y > boidCPUCoords[3]) && (boids[i].position.x > boidCPUCoords[2])) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[4];
			}
		} else if (neighbouringBoidCPUs[6] != 0) {
			if ((boids[i].position.y > boidCPUCoords[3]) && (boids[i].position.x < boidCPUCoords[0])) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[6];
			}
		} else if (neighbouringBoidCPUs[1] != 0) {
			if (boids[i].position.y < boidCPUCoords[1]) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[1];
			}
		} else if (neighbouringBoidCPUs[3] != 0) {
			if (boids[i].position.x > boidCPUCoords[2]) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[3];
			}
		} else if (neighbouringBoidCPUs[5] != 0) {
			if (boids[i].position.y > boidCPUCoords[3]) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[5];
			}
		} else if (neighbouringBoidCPUs[7] != 0) {
			if (boids[i].position.x < boidCPUCoords[0]) {
				boidToTransfer = boids[i];
				recipientBoidCPU = neighbouringBoidCPUs[7];
			}
		}
	}

	std::cout << "-Transferring boid #" << boidToTransfer.id <<
		" to boidCPU #" << recipientBoidCPU << std::endl;
}

void updateDisplay() {
	std::cout << "-Updating display" << std::endl;
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

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data) {
	outputData[CMD_LEN]  = len + CMD_HEADER_LEN;
	outputData[CMD_TO]   = to;
	outputData[CMD_FROM] = boidCPUID;
	outputData[CMD_TYPE] = type;

	if (len > 0) {
		createOutputCommandLoop: for(int i = 0; i < len; i++) {
			outputData[CMD_HEADER_LEN + i] = data[i];
		}
	}

	outputAvailable = true;
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void printCommand(bool send, uint32 *data) {
	if(send) {
		if(data[CMD_TO] == CMD_BROADCAST) {
			std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to controller: ";
		} else {
			std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to " << outputData[CMD_TO] << ": ";
		}
	} else {
		if(data[CMD_TO] == CMD_BROADCAST) {
			std::cout << "<- RX, BoidCPU #" << boidCPUID << " received broadcast from controller: ";
		} else {
			std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from " << outputData[CMD_FROM] << ": ";
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
		case CMD_NBR_REQUEST:
			std::cout << "supply boids to neighbour";
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
		case MODE_DRAW:
			std::cout << "send boids to BoidGPU";
			break;
		case CMD_DRAW_INFO:
			std::cout << "boid info heading to BoidCPU";
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

void acceptBoid(uint32 *boidData) {
	// TODO: Parse the input data to create a boid object
	int boidID = 12;
	Vector boidPosition = Vector(12, 100, 0);
	Vector boidVelocity = Vector(10, -2, 0);
	Boid b = Boid(boidID, boidPosition, boidVelocity);

	boids[boidCount] = b;
	boidCount++;
}

//==============================================================================
// Random ======================================================================
//==============================================================================

// http://en.wikipedia.org/wiki/Linear_feedback_shift_register#Galois_LFSRs
// http://stackoverflow.com/q/17764587
// http://stackoverflow.com/a/5009006
int getRandom(int min, int max) {
	shiftLSFR(&lfsr16, POLY_MASK_16);
	int result = (shiftLSFR(&lfsr16, POLY_MASK_16) ^ shiftLSFR(&lfsr15, POLY_MASK_15));

	return (min + (result % (int)(max - min + 1)));
}

int shiftLSFR(uint16 *lfsr, uint16 mask) {
	unsigned period = 0;

	shiftLFSRLoop: do {
		unsigned lsb = *lfsr & 1;	// Get LSB (i.e., the output bit)
		*lfsr >>= 1;				// Shift register
		if (lsb == 1)				// Only apply toggle mask if output bit is 1
			*lfsr ^= 0xB400u;		// Apply toggle mask, value has 1 at bits
									//  corresponding to taps, 0 elsewhere
		++period;
	} while (*lfsr != mask);

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

	position = Vector(0, 0, 0);
	velocity = Vector(0, 0, 0);

	neighbouringBoidsCount = 0;
}

Boid::Boid(int _boidID, Vector initPosition, Vector initVelocity) {
	id = _boidID;

	position = initPosition;
	velocity = initVelocity;

	neighbouringBoidsCount = 0;

	std::cout << "Created boid #" << id << std::endl;
	printBoidInfo();
}

// TODO: Will need to make a copy of the neighbours rather than a reference so
//	that as the neighbours are updated, the neighbour list doesn't change
void Boid::calculateNeighbours(Boid *possibleNeighbours, int possibleNeighbourCount) {
	std::cout << "Calculating neighbours for boid #" << id << std::endl;

	calcBoidNbrsLoop: for (int i = 0; i < possibleNeighbourCount; i++) {
		if (possibleNeighbours[i].id != id) {
			double distance = Vector::distanceBetween(position, possibleNeighbours[i].position);
			if (distance < VISION_RADIUS) {
				neighbouringBoids[neighbouringBoidsCount] = &possibleNeighbours[i];
				neighbouringBoidsCount++;
			}
		}
	}

	std::cout << "Boid #" << id << " has " << neighbouringBoidsCount << " neighbours: ";
	printBoidNbsLoop: for (int i = 0; i < neighbouringBoidsCount; i++) {
		std::cout << neighbouringBoids[i]->id << ", ";
	} std::cout << std::endl;
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
		// FIXME: This doesn't work as Boid can't have a list of Boids
		total.add(neighbouringBoids[i]->velocity);
		//total.add(getDummyVector());
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
		// FIXME: This doesn't work as Boid can't have a list of Boids
		diff = Vector::sub(position, neighbouringBoids[i]->position);
		//diff = Vector::sub(position, getDummyVector());
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
		// FIXME: This doesn't work as Boid can't have a list of Boids
		total.add(neighbouringBoids[i]->position);
		//total.add(getDummyVector());
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

//Vector Boid::getVelocity() {
//	return velocity;
//}
//
//Vector Boid::getPosition() {
//	return position;
//}

//uint8 Boid::getID(void) {
//	return id;
//}

//uint8 Boid::getNeighbourCount(void) {
//	return neighbouringBoidsCount;
//}
//
//Boid* Boid::getNeighbours(void) {
//	return &neighbouringBoids;
//}

void Boid::printBoidInfo() {
	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
	std::cout << "Boid Position: [" << position.x << " " << position.y << " " << position.z << "]" << std::endl;
	std::cout << "Boid Velocity: [" << velocity.x << " " << velocity.y << " " << velocity.z << "]" << std::endl;
	std::cout << "===================================" << std::endl;
}

//==============================================================================
// Vector ======================================================================
//==============================================================================

// Constructors ////////////////////////////////////////////////////////////////
Vector::Vector() {
	x = 0;
	y = 0;
	z = 0;
}

Vector::Vector(int12 x_, int12 y_, int12 z_) {
	x = x_;
	y = y_;
	z = z_;
}

// Basic Operations ////////////////////////////////////////////////////////////
void Vector::add(Vector v) {
	x = x + v.x;
	y = y + v.y;
	z = z + v.z;
}

void Vector::sub(Vector v) {
	x = x - v.x;
	y = y - v.y;
	z = z - v.z;
}

void Vector::mul(uint8 n) {
	x = x * n;
	y = y * n;
	z = z * n;
}

void Vector::div(uint8 n) {
	if (n != 0) {
		x = x / n;
		y = y / n;
		z = z / n;
	}
}

// Static Operations /////////////////////////////////////////////////////////
Vector Vector::add(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
	return v3;
}

Vector Vector::sub(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	return v3;
}

// FIXME: The sqrt takes a double, probably will be expensive in h/w
double Vector::distanceBetween(Vector v1, Vector v2) {
	int12 xPart = v1.x - v2.x;
	int12 yPart = v1.y - v2.y;
	int12 zPart = v1.z - v2.z;

	xPart = xPart * xPart;
	yPart = yPart * yPart;
	zPart = zPart * zPart;

	return sqrt(double(xPart + yPart + zPart));
}

bool Vector::equal(Vector v1, Vector v2) {
	if ((v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z)) {
		return true;
	} else {
		return false;
	}
}

// Advanced Operations /////////////////////////////////////////////////////////
uint8 Vector::mag() {
	return (uint8)round(sqrt(double(x*x + y*y + z*z)));
}

void Vector::setMag(uint8 mag) {
	normalise();
	mul(mag);
}

void Vector::normalise() {
	uint8 m = mag();
	div(m);
}

void Vector::limit(uint8 max) {
	if(mag() > max) {
		normalise();
		mul(max);
	}
}

void Vector::bound(uint8 n) {
	// TODO: The is technically not binding the speed, which is the magnitude
	if(x > n) x = n;
	if(y > n) y = n;
	if(z > n) z = n;
}

bool Vector::empty() {
	bool result = true;

	if(x) result = false;
	else if(y) result = false;
	else if(z) result = false;

	return result;
}

// Other ///////////////////////////////////////////////////////////////////////
//std::ostream& operator <<(std::ostream& os, const Vector& v) {
//	os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
//	return os;
//}
