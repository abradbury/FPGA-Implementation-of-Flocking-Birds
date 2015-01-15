#include "boidCPU.h"

#define MAX_CMD_LEN			10		// TODO: Decide on appropriate value
#define MAX_CMD_BODY_LEN	20
#define CMD_HEADER_LEN		4

#define MAX_NEIGHBOUR_BOIDCPUS	8

#define CMD_PING		1	// Controller asking how many locations their are
#define CMD_KILL		2	// Controller stopping the simulation
#define CMD_PING_REPLY	3	// Location response to controller ping
#define CMD_INIT		4	// Controller initiation command
#define CMD_BEGIN 		5	// Begin the simulation
#define CMD_LOAD_INFO	6	// Each location reports its current load
#define CMD_LOAD_ACT	7	// The decision of the controller based on the load
#define CMD_LOC_UPDATE	8	// The new parameters for location if load balanced
#define CMD_BOID		9	// Used to transfer boids between locations

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
	uint32 to = 6;
	uint32 from = 0;
	uint32 dataLength = 0;

	// Test ping response ----------------------------------------------------//
	// 4, 6, 0, 1 ||
//	dataLength = 0;
//	createCommand(dataLength, to, from, CMD_PING, data);

	// Test simulation setup ---------------------------------------------------
	// 14, 6, 0, 4 || 5, 10, 1, 2, 3, 6, 9, 8, 7, 4
	dataLength = 10;
	uint32 newID = 5;
	uint32 initialBoidCount = 10;
	uint32 neighbours[MAX_NEIGHBOUR_BOIDCPUS] = {1, 2, 3, 6, 9, 8, 7, 4};

	data[0] = newID;
	data[1] = initialBoidCount;

	for (int i = 0; i < MAX_NEIGHBOUR_BOIDCPUS; i++) {
		data[2 + i] = neighbours[i];
	}

	createCommand(dataLength, to, from, CMD_INIT, data);

	// Send and receive data ---------------------------------------------------
	printTestBenchCommand(true);

	cmdOut: for(int i = 0; i < command[0]; i++) {
		to_hw.write(command[i]);
	}

	topleveltwo(to_hw, from_hw);

	command[0] = from_hw.read();
	cmdIn: for (int i = 1; i < command[0]; i++) {
		command[i] = from_hw.read();
	}

	printTestBenchCommand(false);

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
	command[0] = len + CMD_HEADER_LEN;
	command[1] = to;
	command[2] = from;
	command[3] = type;

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
		if(command[1] == 0) {
			std::cout << "-> TX, Controller sent broadcast: ";
		} else {
			std::cout << "-> TX, Controller sent command to " << command[1] << ": ";
		}
	} else {
		if(command[1] == 0) {
			std::cout << "<- RX, Controller received broadcast from " << command[2] << ": ";
		} else {
			std::cout << "<- RX, Controller received command from " << command[2] << ": ";
		}
	}

	switch(command[3]) {
		case(0):
			std::cout << "do something";
			break;
		case CMD_PING:
			std::cout << "location ping";
			break;
		case CMD_KILL:
			std::cout << "kill simulation";
			break;
		case CMD_PING_REPLY:
			std::cout << "location ping response";
			break;
		case CMD_INIT:
			std::cout << "initialise location";
			break;
		case CMD_BEGIN:
			std::cout << "begin the simulation";
			break;
		case CMD_LOAD_INFO:
			std::cout << "location load information";
			break;
		case CMD_LOAD_ACT:
			std::cout << "load-balancing decision";
			break;
		case CMD_LOC_UPDATE:
			std::cout << "new location parameters";
			break;
		case CMD_BOID:
			std::cout << "boid";
			break;
		default:
			std::cout << "UNKNOWN COMMAND";
			break;
	}

	std::cout << std::endl;

//	if(cdbg) {
		std::cout << "\t";
		for(int i = 0; i < CMD_HEADER_LEN; i++) {
			std::cout << command[i] << " ";
		}

		std::cout << "|| ";

		for(int i = 0; i < command[0] - CMD_HEADER_LEN; i++) {
			std::cout << command[CMD_HEADER_LEN + i] << " ";
		}
		std::cout << std::endl;
//	}
}



