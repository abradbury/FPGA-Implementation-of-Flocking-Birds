#include "location.h"

#define PARAMCOUNT 3

#define FIDLEN 			4	// The length of the FPGA ID
#define MAXNEIGHBOURS 	8	// The maximum number of neighbouring locations

#define CMD_HEADER_LEN	5	//
#define MAX_CMD_BODY_LEN 20	//
#define MAX_CMD_LEN		CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define CMD_PING		1	// Controller asking how many locations their are
#define CMD_KILL		2	// Controller stopping the simulation
#define CMD_PING_REPLY	3	// Location response to controller ping
#define CMD_INIT		4	// Controller initiation command

#define BROADCAST		0	// Used by the controller to address all locations

// Function headers
void createCommandController(uint32 *command, uint32 to, uint32 type, uint32 *data);
void printCommandController(uint32* command, bool send);

// Globals
// TODO: Should this be placed somewhere else? A #define maybe?
uint8 cID;

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
     *
     * Command types:
     * 	1	Ping
     * 	2	Kill
     * 	3	Ping reply
     */

    // This value is to be specified by the user.
    uint32 numberOfBoids = 90;

    cID = 0;

    // Write command -----------------------------------------------------------
    uint32 command[MAX_CMD_LEN];
    uint32 data[MAX_CMD_BODY_LEN];

    createCommandController(command, BROADCAST, CMD_PING, data);

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

	// Read command ------------------------------------------------------------
	// TODO: Implement read
	// -------------------------------------------------------------------------






//    // The controller broadcasts a ping over the network.
//    // TODO: Decide on the networking structure
//    uint32 command[MAX_CMD_LEN] = {0, 0, CMD_PING, 0, 0};
//    for(int i = 0; i < CMD_HEADER_LEN; i++) {
//		to_hw.write(command[i]);
//	}

//    //////////////////////////////////
//    //////// Run the hardware ////////
//    //////////////////////////////////
//	toplevel(to_hw, from_hw);
//	//////////////////////////////////
//
//    // Each location responds with the ID of the FPGA that it is located on and
//    // a randomly generated temporary ID.
//    // TODO: Have a structure for assigning locations against an ID
//	////////////////////////////////////////////////////////////////////////////
//	// Read the command header
//	cmdHeadIn: for(int i = 0; i < CMD_HEADER_LEN; i++) {
//		command[i] = from_hw.read();
//		std::cout << command[i] << " ";
//	} std::cout << "|| ";
//
//	// Read in the command body
//	cmdBodyIn: for(int i = 0; i < command[3]; i++) {
//		command[CMD_HEADER_LEN + i] = from_hw.read();
//		std::cout << command[CMD_HEADER_LEN + i] << " ";
//	} std::cout << std::endl;
//	////////////////////////////////////////////////////////////////////////////
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

void createCommandController(uint32 *command, uint32 to, uint32 type, uint32 *data) {
	command[0] = to;
	command[1] = cID;
	command[2] = type;
	command[3] = sizeof(data)/sizeof(data[0]);

	dataToCmd: for(int i = 0; i < command[3]; i++) {
		command[4 + i] = data[i];
	}
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void printCommandController(uint32 *command, bool send) {
	if(send) {
		if(command[0] == 0) {
			std::cout << "Controller(1) sent broadcast: ";
		} else {
			std::cout << "Controller(1) sent command to " << command[0] << ": ";
		}
	} else {
		if(command[0] == 0) {
			std::cout << "Controller(1) received broadcast from " << command[1] << ": ";
		} else {
			std::cout << "Controller(1) received command from " << command[1] << ": ";
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
		default:
			std::cout << "UNKNOWN COMMAND";
			break;
	}

	std::cout << std::endl;
}
