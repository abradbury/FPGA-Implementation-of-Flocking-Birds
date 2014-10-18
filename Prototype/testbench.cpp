#include "toplevel.h"

#define PARAMLEN 3

int main() {
    hls::stream<uint32> to_hw, from_hw;

    ///////////////////////////////////
    // Number of boids, grid width, grid height
    uint32 paramData[PARAMLEN] = {10, 15, 15};

    // Write parameter data
	for(int i = 0; i < PARAMLEN; i++) {
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
}
