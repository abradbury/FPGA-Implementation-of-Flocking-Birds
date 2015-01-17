#include "boidCPU.h"

#define MAX_CMD_LEN			10		// TODO: Decide on appropriate value
#define MAX_CMD_BODY_LEN	20
#define CMD_HEADER_LEN		4

#define EDGE_COUNT				4		// The number of edges a BoidCPU has
#define MAX_BOIDCPU_NEIGHBOURS	8		// The max neighbours a BoidCPUs has

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

// Globals
uint32 command[MAX_CMD_LEN];

// Function headers
void printTestBenchCommand(bool send);
void createCommand(uint32 len, uint32 to, uint32 from, uint32 type, uint32 *data);

/**
 * This acts as the external interface to the BoidCPU
 */
int main() {
	hls::stream<uint32> to_hw, from_hw;

	uint32 data[MAX_CMD_BODY_LEN];
	uint32 to;
	uint32 from = 0;
	uint32 dataLength = 0;

	// Boolean array depending on whether the controller is expecting a response
	bool expectResponse[CMD_KILL];
	expectResponse[MODE_INIT] 		= false;
	expectResponse[CMD_PING] 		= true;
	expectResponse[CMD_PING_REPLY] 	= false;
	expectResponse[CMD_USER_INFO] 	= false;
	expectResponse[CMD_SIM_SETUP] 	= false;
	expectResponse[MODE_CALC_NBRS] 	= false;
	expectResponse[CMD_NBR_REQUEST] = true;
	expectResponse[CMD_NBR_REPLY] 	= false;
	expectResponse[MODE_POS_BOIDS] 	= false;
	expectResponse[CMD_LOAD_BAL] 	= true;
	expectResponse[MODE_TRAN_BOIDS] = false;
	expectResponse[MODE_DRAW]		= false;
	expectResponse[CMD_DRAW_INFO] 	= false;
	expectResponse[CMD_KILL] 		= true;

	// Test ping response ----------------------------------------------------//
	// 4, 0, 0, 1 ||
//	to = CMD_BROADCAST;
//	dataLength = 0;
//	createCommand(dataLength, to, from, CMD_PING, data);

	// Test simulation setup ---------------------------------------------------
	// 18, 6, 0, 4 || 5, 10, 480, 240, 720, 480, 1, 2, 3, 6, 9, 8, 7, 4
//	dataLength = 14;
//	to = 93;			// The current random ID of the test BoidCPU
//
//	uint32 newID = 6;
//	uint32 initialBoidCount = 10;
//	uint32 coords[EDGE_COUNT] = {480, 240, 720, 480};
//	uint32 neighbours[MAX_BOIDCPU_NEIGHBOURS] = {1, 2, 3, 6, 9, 8, 7, 4};
//
//	data[0] = newID;
//	data[1] = initialBoidCount;
//
//	for (int i = 0; i < EDGE_COUNT; i++) {
//		data[2 + i] = coords[i];
//	}
//
//	for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
//		data[EDGE_COUNT + 2 + i] = neighbours[i];
//	}
//
//	createCommand(dataLength, to, from, CMD_SIM_SETUP, data);

	// Test starting the simulation
	dataLength = 0;
	to = CMD_BROADCAST;
	createCommand(dataLength, to, from, MODE_CALC_NBRS, data);

	// Send and receive data ---------------------------------------------------
	printTestBenchCommand(true);

	cmdOut: for(int i = 0; i < command[CMD_LEN]; i++) {
		to_hw.write(command[i]);
	}

	topleveltwo(to_hw, from_hw);

	if (expectResponse[command[CMD_TYPE]] == true) {
		command[CMD_LEN] = from_hw.read();

		cmdIn: for (int i = 0; i < command[CMD_LEN] - 1; i++) {
			command[i + 1] = from_hw.read();
		}

		printTestBenchCommand(false);
	}

	// Test

	// Power on system
	// BoidCPUs do internal setup
	// BoidCPUs wait for ping from controller
	// BoidCPUs respond to ping from controller
	// BoidCPUs wait for setup information from controller
	// BoidCPUs initialise using setup information
	//
	// BoidCPUs wait for 'go' command
	//
	// BoidCPUs ask for boid data from neighbouring BoidCPUs
	// BoidCPUs calculate the neighbours for each boid
	//
	// BoidCPUs calculate the next positions of their boids
	//
	// BoidCPUs signal to the controller if they are overloaded
	// Otherwise, they wait to see if any load balancing is to be performed
	// Load balancing is performed - TODO
	//
	// BoidCPUs transfer boids beyond their bounds to neighbouring BoidCPUs
	// TODO: Determine how this will be done...
	//
	// BoidCPUs supply the graphical component with the new boid positions
	//

	// System state?

	//
	// Setup BoidCPU
	// BoidCPU waits for go command (blocking on read)
	// On go, BoidCPU calculates new positions
	// Outputs if overloaded, else waits for load balancing
	//

	return 0;
}

void createCommand(uint32 len, uint32 to, uint32 from, uint32 type, uint32 *data) {
	command[CMD_LEN]  = len + CMD_HEADER_LEN;
	command[CMD_TO]   = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		dataToCmd: for(int i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}
}

/**
 * Parses the supplied command and prints it out to the terminal
 *
 * FIXME: Testbench file cannot have same method name
 */
void printTestBenchCommand(bool send) {
	if(send) {
		if(command[CMD_TO] == CMD_BROADCAST) {
			std::cout << "-> TX, Controller sent broadcast: ";
		} else {
			std::cout << "-> TX, Controller sent command to " << command[CMD_TO] << ": ";
		}
	} else {
		if(command[CMD_TO] == CMD_BROADCAST) {
			std::cout << "<- RX, Controller received broadcast from " << command[CMD_FROM] << ": ";
		} else {
			std::cout << "<- RX, Controller received command from " << command[CMD_FROM] << ": ";
		}
	}

	switch(command[CMD_TYPE]) {
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
	for(int i = 0; i < CMD_HEADER_LEN; i++) {
		std::cout << command[i] << " ";
	}

	std::cout << "|| ";

	for(int i = 0; i < command[CMD_LEN] - CMD_HEADER_LEN; i++) {
		std::cout << command[CMD_HEADER_LEN + i] << " ";
	}
	std::cout << std::endl;
}

