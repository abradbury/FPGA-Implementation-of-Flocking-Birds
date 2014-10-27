#include "location.h"

#define PARAMCOUNT 3

#define CMDLEN 4		// The length of the system commands
#define FIDLEN 4		// The length of the FPGA ID

int main() {
    hls::stream<uint32> to_hw, from_hw;

    /**
     * This test bench needs to essentially act as the controller and all the
     * other locations by sending information to the location under test,
     * receiving and processing any responses.
     */

    // This value is to be specified by the user.
    uint32 numberOfBoids = 100;

    // The controller broadcasts a ping over the network.
    // TODO: Decide on the networking structure
    uint32 pingCommand[CMDLEN] = {1, 0, 1, 1};
    for(int i = 0; i < CMDLEN; i++) {
		to_hw.write(pingCommand[i]);
	}

    // Each location responds with the ID of the FPGA that it is located on.
    // TODO: Have a structure for assigning locations against an ID
    uint32 serialNumber[FIDLEN];
    for(int i = 0; i < FIDLEN; i++) {
    	serialNumber[i] = from_hw.read();
	}

    // This would be determined based on the number of ping responses.
    uint32 numberOfLocations = 10;

    // The controller uses the ping responses and serial numbers to determine
    // how to divide the number of boids across the locations and how to
    // position the locations in the environment.




    ///////////////////////////////////
    // Number of boids, grid width, grid height
    uint32 paramData[] = {10, 15, 15};

    // Write the number of parameters
    to_hw.write(PARAMCOUNT);

    // Then write the parameters themselves
	for(int i = 0; i < PARAMCOUNT; i++) {
		to_hw.write(paramData[i]);
	}

    //////////////////////////////////

    // Run the hardware
    toplevel(to_hw, from_hw);

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
