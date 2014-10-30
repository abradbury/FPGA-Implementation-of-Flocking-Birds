#include "location.h"

#define SIZEOF_ARRAY( a ) (sizeof( a ) / sizeof( a[ 0 ] ))

#define CMD_HEADER_LEN	5	//
#define MAX_CMD_BODY_LEN 20	//
#define MAX_CMD_LEN		CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define MAX_NEIGHBOURS 	8	// The maximum number of neighbouring locations
#define MAX_LOCATIONS 	100	// The maximum number of location allowed

#define CMD_PING		1	// Controller asking how many locations their are
#define CMD_KILL		2	// Controller stopping the simulation
#define CMD_PING_REPLY	3	// Location response to controller ping
#define CMD_INIT		4	// Controller initiation command
#define CMD_BEGIN 		5	// Begin the simulation
#define CMD_LOAD_INFO	6	// Each location reports its current load
#define CMD_LOAD_ACT	7	// The decision of the controller based on the load
#define CMD_LOC_UPDATE	8	// The new parameters for location if load balanced
#define CMD_BOID		9	// Used to transfer boids between locations

#define BROADCAST		0	// Used by the controller to address all locations

// Other
struct Location {
	uint32 id;
	uint32 fpga;
};

// Function headers
void createCommandController(uint32 *command, uint32 to, uint32 type, uint32 len, uint32 *data);
void printCommandController(uint32* command, bool send);

// Globals
// TODO: Should this be placed somewhere else? A #define maybe?
uint8 cID;

bool cdbg;

int main() {
    hls::stream<uint32> to_hw, from_hw;

    /**
     * This test bench needs to essentially act as the controller and all the
     * other locations by sending information to the location under test,
     * receiving and processing any responses.
     *
     * There are 9 locations across 3 FPGAs, distributed as follows:
     * 		FPGA1 (120): Controller, L1, L2
     * 		FPGA2 (121): L3, L4, L5, L6
     * 		FPGA3 (122): L7, L8, L9
     * The location under test is location #6
     *
     * Command header: {to, from, type, length, N/A}
     * 		If to is 0, it is a broadcast
     * 		The controller has ID 1
     */

    // Controller initialisation -----------------------------------------------
    uint32 numberOfBoids = 90;		// Specified by the user
    cID = 1;						// Set the controller ID

    cdbg = true;					// True to print out the command packets
    // -------------------------------------------------------------------------

    // Write command (ping) ----------------------------------------------------
    uint32 command[MAX_CMD_LEN];
    uint32 data[MAX_CMD_BODY_LEN];
    uint32 len = 0;

    createCommandController(command, BROADCAST, CMD_PING, len, data);

    cmdOut: for(int i = 0; i < CMD_HEADER_LEN + command[3]; i++) {
    	to_hw.write(command[i]);
    }

    printCommandController(command, true);
    // -------------------------------------------------------------------------

	//////////////////////////////////
	//////// Run the hardware ////////
	//////////////////////////////////
	toplevel(to_hw, from_hw);
	//////////////////////////////////

	// Read command (ping reply) -----------------------------------------------
	bool ignoreCmd;

	// First read the command header
	cmdHeadIn: for(int i = 0; i < CMD_HEADER_LEN; i++) {
		command[i] = from_hw.read();
	}

	// If the command is not a broadcast and not addressed to me, ignore it,
	// but still have to read the input
	// TODO: Find a way of not having to read the input
	if((command[0] != 0) && (command[0] != cID)) {
		ignoreCmd = true;
		for(int i = 0; i < command[3]; i++) {
			from_hw.read();
		}
	} else {
		// Else, read the command body
		ignoreCmd = false;
		cmdBodyIn: for(int i = 0; i < command[3]; i++) {
			command[CMD_HEADER_LEN + i] = from_hw.read();
		}
	}

	printCommandController(command, false);
	// -------------------------------------------------------------------------

	// Process command (ping reply) --------------------------------------------
	Location locations[MAX_LOCATIONS];
	uint32 numberOfLocations = 0;

	// Add the current location to the array
	// New location IDs are based on their index in the array
	// Insert test location at index 5 (ID = 6)
	numberOfLocations = 5;
	locations[numberOfLocations].id = command[1];
	locations[numberOfLocations].fpga = command[CMD_HEADER_LEN + 0];
	numberOfLocations++;

	// TODO: Handle remainders of division
	numberOfLocations = 9;
	uint32 boidsPerLocation = numberOfBoids / numberOfLocations;

	// TODO: Position locations
	// By giving coordinates of the square corners? Absolute or relative?
	uint32 position[8] = {0, 0, 0, 50, 50, 50, 50, 0};

	// TODO: Generate neighbour lists for locations
	// Use location index as new ID not yet assigned (as old needed to address)
	uint32 neighbours[MAX_NEIGHBOURS] = {1, 2, 3, 5, 9, 8, 7, 4};

	// -------------------------------------------------------------------------

	// Write command (location initialisation) ---------------------------------
	// TODO: Does the command (and data) array need clearing before reuse?
	data[0] = 6;
	data[1] = boidsPerLocation;
	for(int i = 0; i < MAX_NEIGHBOURS; i++) {
		data[2 + i] = neighbours[i];
	}
	for(int i = 0; i < 8; i++) {
		data[2 + MAX_NEIGHBOURS + i] = position[i];
	}
	len = 2 + MAX_NEIGHBOURS + 8;

	createCommandController(command, locations[5].id, CMD_INIT, len, data);

	cmdOutInit: for(int i = 0; i < CMD_HEADER_LEN + command[3]; i++) {
		to_hw.write(command[i]);
	}

	printCommandController(command, true);

	// Now that the locations have their new IDs update the array
	// TODO:  Assign new IDs (based on index of location array)
	locations[5].id = 6;
	// -------------------------------------------------------------------------

	//////////////////////////////////
	//////// Run the hardware ////////
	//////////////////////////////////
	toplevel(to_hw, from_hw);
	//////////////////////////////////

	// Write command (begin) ---------------------------------------------------
	createCommandController(command, BROADCAST, CMD_BEGIN, len, data);

	cmdOutBegin: for(int i = 0; i < CMD_HEADER_LEN + command[3]; i++) {
		to_hw.write(command[i]);
	}

	printCommandController(command, true);
	// -------------------------------------------------------------------------

	//////////////////////////////////
	//////// Run the hardware ////////
	//////////////////////////////////
	toplevel(to_hw, from_hw);
	//////////////////////////////////

	// Read command (load) -----------------------------------------------------
	// TODO: Add main part of controller, init done
	// -------------------------------------------------------------------------


	// FIXME: BIG ISSUE: Every time the toplevel function is called, it does
// 		not remember anything from the previous calls...





//
//	uint32 fpgaID = command[CMD_HEADER_LEN + 1];
//	uint32 tmpLocID = command[1];
//
//    // These would be determined based on the number of ping responses.
//    // TODO: Rework FPGA and location ID storage structure
////    uint32 fpgaIDs[] = {120, 121, 122};
////    uint32 tmpLocationIDs[][] = { {11, 12}, {13, 14, 15, 16}, {17, 18, 19}};
//    uint32 numberOfLocations = 9;
//
//    // The controller calculates the number of boids per location.
//    // TODO: Enhance to handle remainders
//    uint32 boidsPerLocation = numberOfBoids / numberOfLocations;
//
//    // The controller calculates location IDs
////    uint32 locationIDs[][] = { {1, 2}, {3, 4, 5, 6}, {7, 8, 9}};
//
//    // The controller determines the location placement and neighbours
//    uint32 neighbours[MAXNEIGHBOURS] = {3, 4, 5, 7, 8, 9, 1, 2};
//
//    // The controller informs the locations of its calculations
//    // FIXME: Can't put an array in an array (neighbours)
//    uint32 initCommand[] = {16, 0, CMD_INIT, 10, 0, 6, boidsPerLocation, 3, 4, 5, 7, 8, 9, 1, 2};
//    for(int i = 0; i < MAX_CMD_LEN; i++) {
//		to_hw.write(initCommand[i]);
//	}
//
//    //////////////////////////////////
//    //////// Run the hardware ////////
//    //////////////////////////////////
//	toplevel(to_hw, from_hw);
//	//////////////////////////////////

    ///////////////////////////////////
    // Number of boids, grid width, grid height
//    uint32 paramData[] = {10, 15, 15};
//
//    // Write the number of parameters
//    to_hw.write(PARAMCOUNT);
//
//    // Then write the parameters themselves
//	for(int i = 0; i < PARAMCOUNT; i++) {
//		to_hw.write(paramData[i]);
//	}





    // Read and report the output
    //int out = from_hw.read();
    //printf("Output: %d\n", out);
//    printf("Values 1 to %d subtracted from value 0: %d\n", NUMDATA-1, sub);
//
//    //Check values
//    if(sub == 46 && sum == 154) {
//        return 0;
//    } else {
//        return 1; //An error!
//    }

    return 0;
}

/**
 * Create a command to send from the controller to the locations.
 *
 * If the command is a broadcast command, then no data is to be sent.
 *
 * Apparently, it is not possible to find the length of an array, given a
 * pointer to that array. Therefore, the length of the data has to be supplied.
 */
void createCommandController(uint32 *command, uint32 to, uint32 type, uint32 len, uint32 *data) {
	command[0] = to;
	command[1] = cID;
	command[2] = type;
	command[3] = (command[0] == BROADCAST)? (uint32)0 : len;
	command[4] = 0;

	dataToCmd: for(int i = 0; i < command[3]; i++) {
		command[5 + i] = data[i];
	}
}

/**
 * Parses the supplied command and prints it out to the terminal
 *
 * FIXME: Testbench file cannot have same method name
 */
void printCommandController(uint32 *command, bool send) {
	if(send) {
		if(command[0] == 0) {
			std::cout << "-> TX, Controller(1) sent broadcast: ";
		} else {
			std::cout << "-> TX, Controller(1) sent command to " << command[0] << ": ";
		}
	} else {
		if(command[0] == 0) {
			std::cout << "<- RX, Controller(1) received broadcast from " << command[1] << ": ";
		} else {
			std::cout << "<- RX, Controller(1) received command from " << command[1] << ": ";
		}
	}

	switch(command[2]) {
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

	if(cdbg) {
		std::cout << "\t";
		for(int i = 0; i < CMD_HEADER_LEN; i++) {
			std::cout << command[i] << " ";
		}

		std::cout << "|| ";

		for(int i = 0; i < command[3]; i++) {
			std::cout << command[CMD_HEADER_LEN + i] << " ";
		}
		std::cout << std::endl;
	}
}
