/** 
 * Copyright 2015 abradbury
 * 
 * boids.h
 * 
 * This is the header file for an FPGA's gatekeeper component. A gatekeeper 
 * primarily routes messages to and from BoidCPUs. It determines if a received 
 * message should be sent to a BoidCPU residing on an external FPGA over  
 * Ethernet or to a BoidCPU residing on the same FPGA. It is also able to drop 
 * messages if they are not relevant to internal BoidCPUs.  
 * 
 * This file primarily contains #defines that are used in gatekeeper.c to aid 
 * readability of the source code. The #defines in this file are supposed to be 
 * identical to those in boidMaster.h and boidCPU.h. Ideally, there would be no 
 * duplication, but the tools used did not support a common file. 
 * 
 * This component core was developed using the Xilnix Software Development Kit 
 * (SDK) 14.7 and deployed to a MicroBlaze processor on multiple Xilinx  
 * Spartan-6 LX45 FPGAs using Xilinx Platform Studio (XPS) 14.7.
 *
 ******************************************************************************/

#ifndef BOIDS_H_
#define BOIDS_H_

/**************************** Constant Definitions ****************************/

#define LINUX_ENTER_KEY         0x0A  // The ASCII code for '\n'/Line Feed (LF)
#define WINDOWS_ENTER_KEY       0x0D
#define USING_VLAB              0     // 1 if using VLAB, 0 if not

#define BOID_COUNT              20    // The total number of system boids

// Command definitions ---------------------------------------------------------
#define CMD_HEADER_LEN          4   // The length of the command header
#define MAX_CMD_BODY_LEN        30  // The max length of the command body
#define MAX_CMD_LEN             CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define MAX_OUTPUT_CMDS         15  // The number of output commands to buffer
#define MAX_INPUT_CMDS          5   // The number of input commands to buffer

#define CMD_LEN                 0   // The index of the command length
#define CMD_TO                  1   // The index of the command target
#define CMD_FROM                2   // The index of the command sender
#define CMD_TYPE                3   // The index of the command type

#define CMD_BROADCAST           0   // The number for a broadcast command

#define CONTROLLER_ID           1   // The ID of the controller
#define BOIDGPU_ID              2   // The ID of the BoidGPU
#define FIRST_BOIDCPU_ID        3   // The lowest possible BoidCPU ID
#define CMD_MULTICAST           99  // The 'to' value for multicast commands

#define MODE_INIT               1   //
#define CMD_PING                2   // Controller -> BoidCPU
#define CMD_PING_REPLY          3   // BoidCPU -> Controller
#define CMD_USER_INFO           4   // Controller -> BoidGPU
#define CMD_SIM_SETUP           5   // Controller -> Boid[CG]PU
#define MODE_CALC_NBRS          6   //
#define CMD_PING_END            7   // Gatekeeper -> Controller
#define CMD_NBR_REPLY           8   // BoidCPU -> BoidCPU
#define MODE_POS_BOIDS          9   //
#define CMD_LOAD_BAL            10  // TODO: Decide on implementation
#define MODE_TRAN_BOIDS         11  //
#define CMD_BOID                12  // BoidCPU -> BoidCPU
#define MODE_DRAW               14  // TODO: Perhaps not needed?
#define CMD_DRAW_INFO           15  // BoidCPU -> BoidGPU
#define CMD_KILL                16  // Controller -> All
#define CMD_ACK                 17  // All -> Controller
#define CMD_PING_START          18
#define CMD_DEBUG               76

#define CMD_COUNT               19

#define CMD_SETUP_BNBRS_IDX     7   // Neighbouring BoidCPU start index
#define CMD_SETUP_COORD_IDX     2   // Coordinates start index
#define CMD_SETUP_NBCNT_IDX     6   // Distinct BoidCPU neighbour index
#define CMD_SETUP_NEWID_IDX     0   // New BoidCPU ID index
#define CMD_SETUP_BDCNT_IDX     1   // Initial boid count index
#define CMD_SETUP_SIMWH_IDX     15  // The simulation width/height start index

// BoidCPU definitions ---------------------------------------------------------
#define EDGE_COUNT              4   // The number of edges a BoidCPU has
#define MAX_BOIDCPU_NEIGHBOURS  8   // The maximum neighbours a BoidCPUs has
#define MAX_SYSTEM_BOIDCPUS     10  // The maximum number of BoidCPUs

/****************************** Type Definitions ******************************/

typedef enum {
    false, true
} bool;

#endif /* BOIDS_H_ */
