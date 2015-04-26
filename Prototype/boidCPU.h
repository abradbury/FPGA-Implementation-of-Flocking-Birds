#ifndef __BOIDCPU_H_
#define __BOIDCPU_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>     			// For cout
#include <ap_int.h>                 // For arbitrary precision types
#include <ap_fixed.h>               // For fixed point data types
#include <hls_stream.h>
#include "hls_math.h"               // For sqrt

// Command definitions ---------------------------------------------------------
#define CMD_HEADER_LEN          4   // The length of the command header
#define MAX_CMD_BODY_LEN        30  // The max length of the command body
#define MAX_CMD_LEN             CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define MAX_OUTPUT_CMDS         12  // The number of output commands to buffer
#define MAX_INPUT_CMDS          1   // The number of input commands to buffer

#define CMD_LEN                 0   // The index of the command length
#define CMD_TO                  1   // The index of the command target
#define CMD_FROM                2   // The index of the command sender
#define CMD_TYPE                3   // The index of the command type

#define CMD_BROADCAST           0   // The number for a broadcast command
#define CONTROLLER_ID           1   // The ID of the controller
#define BOIDGPU_ID              2   // The ID of the BoidGPU
#define FIRST_BOIDCPU_ID		3	// The lowest possible BoidCPU ID
#define CMD_MULTICAST           99  // The 'to' value for multicast commands

#define BOID_DATA_LENGTH        3   // The number of bits to send for a boid

#define MODE_INIT               1   // Controller -> BoidCPU (Broadcast (B))
#define CMD_PING                2   // Controller -> BoidCPU (B)
#define CMD_PING_REPLY          3   // BoidCPU -> Controller (Direct (D))
#define CMD_USER_INFO           4   // Controller -> BoidGPU (D)
#define CMD_SIM_SETUP           5   // Controller -> Boid[CG]PU (D)
#define MODE_CALC_NBRS          6   // Controller -> BoidCPU (B)
#define CMD_PING_END			7	// Gatekeeper -> Controller (D)
#define CMD_NBR_REPLY           8   // BoidCPU -> BoidCPU (D)
#define MODE_POS_BOIDS          9   // Controller -> BoidCPU (B)
#define MODE_LOAD_BAL           10  // Controller -> BoidCPU (?)
#define MODE_TRAN_BOIDS         11  // Controller -> BoidCPU (B)
#define CMD_BOID                12  // BoidCPU -> BoidCPU (D)
#define MODE_DRAW               14  // Controller -> BoidCPU (B)
#define CMD_DRAW_INFO           15  // BoidCPU -> BoidGPU (D)
#define CMD_KILL                16  // Controller -> All (B)
#define CMD_ACK					17
#define CMD_PING_START			18
#define CMD_LOAD_BAL_REQUEST	19
#define CMD_LOAD_BAL			20
#define CMD_BOUNDS_AT_MIN		21
#define CMD_DEBUG				76

#define CMD_SETUP_BNBRS_IDX 	7	// Neighbouring BoidCPU start index
#define CMD_SETUP_COORD_IDX 	2	// Coordinates start index
#define CMD_SETUP_NBCNT_IDX 	6	// Distinct BoidCPU neighbour index
#define CMD_SETUP_NEWID_IDX 	0	// New BoidCPU ID index
#define CMD_SETUP_BDCNT_IDX 	1	// Initial boid count index
#define CMD_SETUP_SIMWH_IDX		15	// The simulation width/height start index

// Boid definitions ------------------------------------------------------------
#define MAX_BOIDS               30  // The maximum number of boids for a BoidCPU
#define MAX_VELOCITY            5
#define MAX_FORCE               1   // Determines how quickly a boid can turn
#define VISION_RADIUS           80  // How far a boid can see
#define VISION_RADIUS_SQUARED	6400
#define SEP_RAIDUS_SQUARED		1600
#define MAX_NEIGHBOURING_BOIDS  45  // TODO: Decide on appropriate value?

//#define ALIGNMENT_WEIGHT		1
//#define SEPARATION_WEIGHT		1
//#define COHESION_WEIGHT			1

// BoidCPU definitions ---------------------------------------------------------
#define EDGE_COUNT              4   // The number of edges a BoidCPU has
#define MAX_BOIDCPU_NEIGHBOURS  8   // The maximum neighbours a BoidCPUs has
#define MAX_QUEUED_BOIDS 		10	// The maximum queued boids that can be held

#define BOID_THRESHOLD			30	// The number of boids to overload a BoidCPU

#define X_MIN                   0   // Coordinate index of the min x position
#define Y_MIN                   1   // Coordinate index of the min y position
#define X_MAX                   2   // Coordinate index of the max x position
#define Y_MAX                   3   // Coordinate index of the max y position

#define NORTHWEST 				0	// Index of the BoidCPU to the northwest
#define NORTH					1	// Index of the BoidCPU to the north
#define NORTHEAST				2	// Index of the BoidCPU to the northeast
#define EAST					3	// Index of the BoidCPU to the east
#define SOUTHEAST				4	// Index of the BoidCPU to the southeast
#define SOUTH					5	// Index of the BoidCPU to the south
#define SOUTHWEST				6	// Index of the BoidCPU to the southwest
#define WEST					7	// Index of the BoidCPU to the west

#define NORTH_IDX	12			// The index of the north edge change (load bal)
#define EAST_IDX	8			// The index of the east edge change (load bal)
#define SOUTH_IDX	4			// The index of the south edge change (load bal)
#define WEST_IDX	0			// The index of the west edge change (load bal)


// Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;

typedef ap_uint<16> uint16;
typedef ap_int<16> int16;

typedef ap_int<12> int12;   // Used to represent position and negative velocity
typedef ap_uint<11> uint11;	// For simulation width and height

typedef ap_uint<8> uint8;
typedef ap_int<8> int8;

typedef ap_uint<4> uint4;
typedef ap_int<4> int4;

// 16-bit signed word with 4 fractional bits, truncation and saturation
typedef ap_fixed<16,12, AP_TRN, AP_SAT> int16_fp;

// 32-bit signed word with 8 fractional bits, truncation and saturation
typedef ap_fixed<32,24, AP_TRN, AP_SAT> int32_fp;

// Prototypes
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output);

// Classes
class Vector {
 public:
	int16_fp x;
	int16_fp y;

    Vector();
    Vector(int16_fp x_, int16_fp y_);

    void add(Vector v);
    void mul(int16_fp n);
    void div(int16_fp n);

    int16_fp mag();
    void setMag(int16_fp mag);
    void normalise();

    static Vector sub(Vector v1, Vector v2);
    static int32_fp squaredDistanceBetween(Vector v1, Vector v2);
};

class Boid {
 public:
    Vector position;        	// The current pixel position of the boid
    Vector velocity;        	// The current velocity of the boid
    uint16 id;              	// TODO: Remove this on deployment - MAYBE

    Boid();
    Boid(uint16 _boidID, Vector initPosition, Vector initVelocity);

    void update();                  // Calculate the boid's new position
    void draw();                    // Draw the boid (send to BoidGPU)

    void printBoidInfo();

    // Tell the boid where its neighbours are located in the neighbouring boid
    // list held by the BoidCPU and how many there are
    void setNeighbourDetails(uint8 index, uint8 count);

 private:
    Vector acceleration;

    // Points to this boid's list of neighbouring boids in the list of boid
    // neighbouring boids that is stored by the boid's BoidCPU
    uint8 boidNeighbourIndex;
    uint8 boidNeighbourCount;

    Vector align();         // Calculate the alignment force
    Vector separate();      // Calculate the separation force
    Vector cohesion();      // Calculate the cohesion force
};

#endif
