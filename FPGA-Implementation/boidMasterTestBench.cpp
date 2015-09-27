/** 
 * Copyright 2015 abradbury
 * 
 * boidMasterTestBench.cpp
 * 
 * This file acts as a test bench for the BoidMaster FPGA core. The BoidMaster 
 * is a controller that is primarily responsible for dealing with any load  
 * balancing requests, synchronising the simulation and the initial setup of  
 * the simulation system.
 * 
 * The test bench is able to simulate messages send from BoidCPUs to the 
 * BoidMaster.
 * 
 * This FPGA core was developed using the 2013.4 version Xilinx’s Vivado High 
 * Level Synthesis (HLS) Design Suite.
 *
 * TODO: Towards the end of the project the test bench was used less and may 
 *          have become disjont from the FPGA cores
 *
 ******************************************************************************/

/******************************* Include Files ********************************/

#include "boidMaster.h"

/**************************** Constant Definitions ****************************/

#define TB_MAX_OUTPUT_CMDS  30

/**************************** Variable Definitions ****************************/

uint32 tbOutputData[TB_MAX_OUTPUT_CMDS][MAX_CMD_LEN];
uint32 tbInputData[MAX_INPUT_CMDS][MAX_CMD_LEN];
uint32 tbOutputCount = 0;
uint32 tbInputCount = 0;

uint32 tbData[MAX_CMD_BODY_LEN];
uint32 tbTo;
uint32 tbFrom;
uint32 tbDataLength = 0;

uint32 masterGatekeeerID = -971895691;

uint32 tbGatekeeperCount;
uint32 tbGatekeeperIDs[8];

/**************************** Function Prototypes *****************************/

void simulateAck(uint32 from, uint32 type);

void simulatePingStart();
void simulateUserInfo();
void issueEndOfPing();
void simulatePingReplies();
void simulateOverloadedBoidCPU();

void simulateSetupAck();
void simulateNbrSearchAck();
void simulatePositionBoidsAck();
void simulateLoadBalanceAck();
void simulateBoidTransferAck();
void simulateBoidGPUAck();

void processSetupInfo();

void tbPrintCommand(bool send, uint32 *data);
void tbCreateCommand(uint32 len, uint32 to, uint32 from, uint32 type,
        uint32 *data);

/******************************************************************************/
/*
 * This is the main method of the test bench and connects to the BoidMaster 
 * FPGA core that is under test. 
 * 
 * Firstly, the messages/commands to be sent to the BoidMaster are constructed 
 * and stored in an output array. Then these messages are written/sent to the 
 * BoidMaster. When the test bench has finished sending the messages the 
 * BoidMaster core is called and it deals with the sent messages. The test 
 * bench then waits for replies and deals with them. 
 * 
 * Note that it is not possible to then send more data to the BoidMaster and  
 * for the BoidMaster to retain its state from the previous messages. 
 * 
 * TODO: Add support for multiple time steps, if possible
 *
 * @param   None
 *
 * @return          0 if success, 1 if failure
 *
 ******************************************************************************/
int main() {
    hls::stream<uint32> to_hw, from_hw;

    // Test BoidMaster input ---------------------------------------------------
    // Comment in/out commands as appropriate
    simulatePingStart();
    simulatePingReplies();
    issueEndOfPing();

    simulateUserInfo();
    simulateSetupAck();

    simulateNbrSearchAck();
    simulatePositionBoidsAck();
    simulateBoidTransferAck();

    simulateOverloadedBoidCPU();
    simulateLoadBalanceAck();

    // simulateBoidGPUAck();

    // Send data ---------------------------------------------------------------
    outerOutputLoop: for (int i = 0; i < tbOutputCount; i++) {
        tbPrintCommand(true, tbOutputData[i]);
        innerOutputLoop: for (int j = 0; j < tbOutputData[i][CMD_LEN]; j++) {
            to_hw.write(tbOutputData[i][j]);
        }
    }

    // TODO: Come up with a better buffer counter update method
    tbOutputCount = 0;

    std::cout << "======TestBench finished sending======" << std::endl;

    // Run the hardware --------------------------------------------------------
    boidMaster(to_hw, from_hw);

    // Receive data ------------------------------------------------------------
    bool inputAvailable = from_hw.read_nb(tbInputData[tbInputCount][CMD_LEN]);

    while (inputAvailable) {
        inLp: for (int i = 0; i < tbInputData[tbInputCount][CMD_LEN] - 1; i++) {
            tbInputData[tbInputCount][i + 1] = from_hw.read();
        }

        tbPrintCommand(false, tbInputData[tbInputCount]);

        // Process data --------------------------------------------------------
        // Comment in/out as appropriate
        switch (tbInputData[tbInputCount][CMD_TYPE]) {
            // case CMD_PING:
            //     processPing();
            //     break;
            case CMD_SIM_SETUP:
                processSetupInfo();
                break;
            // case MODE_CALC_NBRS:
            //     processCalcNeighbours();
            //     break;
            // case MODE_POS_BOIDS:
            //     processBoidPositioning();
            //     break;
            // case MODE_TRAN_BOIDS:
            //     processBoidTransfer();
            //     break;
            default:
                break;
        }

        // Check for more input ------------------------------------------------
        // Don't really need input buffer if data is processed before the next
        // inputCount++;
        inputAvailable = from_hw.read_nb(tbInputData[tbInputCount][CMD_LEN]);
    }

    std::cout << "=====TestBench finished receiving=====" << std::endl;

    return 0;   // A non-zero return value signals an error
}

//============================================================================//
// State processing functions ------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Simulate an acknowledgement (ACK) message for a specified message type and 
 * from a specified sender of the ACK. 
 * 
 * @param   from    The ID entity sending the ACK message
 * @param   type    The type of message being acknowledged
 *
 * @return  None
 *
 ******************************************************************************/
void simulateAck(uint32 from, uint32 type) {
    tbData[0] = type;
    tbDataLength = 1;
    tbTo = CONTROLLER_ID;
    tbFrom = from;          // Just some random ID
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_ACK, tbData);
}

/******************************************************************************/
/*
 * Simulate the command from the BoidMaster's gatekeeper informing the 
 * BoidMaster to commence the ping search for available BoidCPUs. The ping 
 * start command is sent from user input to allow time for the gatekeepers to 
 * register with the Ethernet switch. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulatePingStart() {
    std::cout << "Simulating ping start..." << std::endl;
    tbDataLength = 0;
    tbTo = CONTROLLER_ID;
    tbFrom = masterGatekeeerID;
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_START, tbData);
}

/******************************************************************************/
/*
 * Simulate a message from the BoidMaster's gatekeeper containing user-inputted 
 * information. Currently, the number of boids in the system is the only such 
 * piece of user information supported. 
 *
 * TODO: Parameterise the number of boids
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateUserInfo() {
    std::cout << "Simulating user info..." << std::endl;
    tbData[0] = 20;         // Number of boids
    tbDataLength = 1;
    tbTo = CONTROLLER_ID;
    tbFrom = masterGatekeeerID;
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_USER_INFO, tbData);
}

/******************************************************************************/
/*
 * Simulate the replies from the gatekeepers of BoidCPUs to a ping sent from 
 * the BoidMaster. Currently, the replies from 3 gatekeepers are simulated 
 * (including the BoidMaster's gatekeeper). 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulatePingReplies() {
    std::cout << "Simulating ping replies..." << std::endl;
    tbGatekeeperCount = 3;
    tbGatekeeperIDs[0] = masterGatekeeerID;
    tbGatekeeperIDs[1] = 66;
    tbGatekeeperIDs[2] = 432;

    tbData[0] = 2;                  // Number of resident BoidCPUs
    tbDataLength = 1;
    tbTo = CONTROLLER_ID;
    tbFrom = tbGatekeeperIDs[0];    // Random Gatekeeper ID
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_REPLY, tbData);

    tbData[0] = 6;                  // Number of resident BoidCPUs
    tbDataLength = 1;
    tbTo = CONTROLLER_ID;
    tbFrom = tbGatekeeperIDs[1];    // Random Gatekeeper ID
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_REPLY, tbData);

    tbData[0] = 1;                  // Number of resident BoidCPUs
    tbDataLength = 1;
    tbTo = CONTROLLER_ID;
    tbFrom = tbGatekeeperIDs[2];    // Random Gatekeeper ID
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_REPLY, tbData);

    std::cout << "Responding to ping with 6 BoidCPUs (2/4)..." << std::endl;
}

/******************************************************************************/
/*
 * Issue an end of ping command to the BoidMaster. This command would be issued 
 * when the user manually stops the search for new BoidCPUs. Having the user 
 * end the search removes the need for a timer to be included in the FPGA 
 * design. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void issueEndOfPing() {
    std::cout << "Simulating end of ping..." << std::endl;
    tbDataLength = 0;
    tbTo = CONTROLLER_ID;
    tbFrom = masterGatekeeerID;
    tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_END, tbData);
}

/******************************************************************************/
/*
 * Parse the BoidCPU setup information sent by the BoidMaster and print the 
 * data out to the standard output. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void processSetupInfo() {
    std::cout << "Processing setup info..." << std::endl;

    // Store BoidCPU information
    uint32 boidCPUID = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_NEWID_IDX];
    uint32 boidCount = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_BDCNT_IDX];
    uint32 distNbrs  = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_NBCNT_IDX];
    uint32 simWidth  = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX + 0];
    uint32 simHeight = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX + 1];

    uint32 coords[EDGE_COUNT];
    for (int i = 0; i < EDGE_COUNT; i++) {
        coords[i] = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_COORD_IDX + i];
    }

    uint32 nbrs[MAX_BOIDCPU_NEIGHBOURS];
    for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        nbrs[i] = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_BNBRS_IDX + i];
    }

    // Print BoidCPU information
    std::cout << "BoidCPU #" << boidCPUID << " of Gatekeeper #" <<
            tbInputData[0][CMD_TO] << " has an initial boid count of " <<
            boidCount << " coordinates of [";

    for (int i = 0; i < EDGE_COUNT; i++) {
        std::cout << coords[i] << ", ";
    }

    std::cout << "], \n" << distNbrs << " distinct neighbours: [";

    for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        std::cout << nbrs[i] << ", ";
    }

    std:: cout << "] and the simulation size is [" << simWidth << ", " <<
            simHeight << "]" << std::endl;
}

/******************************************************************************/
/*
 * Simulate the acknowledgement (ACK) from each gatekeeper responsible for its 
 * resident BoidCPUs singalling the completion of the setup phase of the 
 * simulation. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateSetupAck() {
    std::cout << "Simulating setup ACK..." << std::endl;
    for (int i = 0; i < tbGatekeeperCount; i++) {
        simulateAck(tbGatekeeperIDs[i], CMD_SIM_SETUP);
    }
}

/******************************************************************************/
/*
 * Simulate the acknowledgement (ACK) from each gatekeeper responsible for its 
 * resident BoidCPUs singalling the completion of the neighbour search phase of 
 * the simulation. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateNbrSearchAck() {
    std::cout << "Simulating neighbour search ACK..." << std::endl;
    for (int i = 0; i < tbGatekeeperCount; i++) {
        simulateAck(tbGatekeeperIDs[i], MODE_CALC_NBRS);
    }
}

/******************************************************************************/
/*
 * Simulate the acknowledgement (ACK) from each gatekeeper responsible for its 
 * resident BoidCPUs singalling the completion of the boid position update 
 * phase of the simulation. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulatePositionBoidsAck() {
    std::cout << "Simulating position boids ACK..." << std::endl;
    for (int i = 0; i < tbGatekeeperCount; i++) {
        simulateAck(tbGatekeeperIDs[i], MODE_POS_BOIDS);
    }
}

/******************************************************************************/
/*
 * Simulate a load balancing request from those BoidCPUs that do need load 
 * balancing.
 * 
 * TODO: Combine with simulateLoadBalanceAck()?
 * TODO: Adjust so that it is parametisable as to which BoidCPU would request 
 *  load balancing and which wouldn't
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateOverloadedBoidCPU() {
    for (int i = 0; i < tbGatekeeperCount; i++) {
//      if (tbGatekeeperIDs[i] != 66) {
//          std::cout << "Simulating load balance ACK..." << std::endl;
//          simulateAck(tbGatekeeperIDs[i], MODE_LOAD_BAL);
//      } else {
            std::cout << "Simulating load balance request..." << std::endl;
            tbTo = CONTROLLER_ID;
            tbFrom = 3;
            tbCreateCommand(0, tbTo, tbFrom, CMD_LOAD_BAL_REQUEST, tbData);
//      }
    }
}

/******************************************************************************/
/*
 * Simulate an acknowledgement (ACK) from those BoidCPUs that are not 
 * overloaded and do not need load balancing. 
 * 
 * TODO: Combine with simulateOverloadedBoidCPU()?
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateLoadBalanceAck() {
    std::cout << "Simulating load balance (2) ACK..." << std::endl;
    for (int i = 0; i < tbGatekeeperCount; i++) {
//      simulateAck(tbGatekeeperIDs[i], CMD_LOAD_BAL);

        if (tbGatekeeperIDs[i] != 432) {
            std::cout << "Simulating load balance ACK..." << std::endl;
            simulateAck(tbGatekeeperIDs[i], CMD_LOAD_BAL);
        }
    }
}

/******************************************************************************/
/*
 * Simulate an acknowledgement (ACK) from each of the gatekeepers in the system 
 * which signals that all their BoidCPUs have finished transfering boids. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateBoidTransferAck() {
    std::cout << "Simulating boid transfer ACK..." << std::endl;
    for (int i = 0; i < tbGatekeeperCount; i++) {
        simulateAck(tbGatekeeperIDs[i], MODE_TRAN_BOIDS);
    }
}

/******************************************************************************/
/*
 * Simulate an acknowledgement (ACK) from the BoidGPU. Would be used when the 
 * BoidGPU has finished receiving (or drawing) all boids so that the system can 
 * move on to the next stage of the simulation.
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateBoidGPUAck() {
    std::cout << "Simulating BoidGPU ACK..." << std::endl;
    simulateAck(BOIDGPU_ID, MODE_DRAW);
}

//============================================================================//
// Message processing functions ----------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Takes data to be transmitted and places it in an queue of data. This queue 
 * is processed, i.e. the elements sent, in the top level method when all the 
 * messages to be sent to the BoidCPU under test have been created. If the 
 * output queue is full, the new data is not added.
 *
 * @param   len     The length of the message body
 * @param   to      The recipient of the message
 * @param   from    The ID of the message sender
 * @param   type    The type of the message (defined in boidMaster.h)
 * @param   data    The message data
 *
 * @return  None
 *
 ******************************************************************************/
void tbCreateCommand(uint32 len, uint32 to, uint32 from, uint32 type, uint32 *data) {
    if (tbOutputCount < TB_MAX_OUTPUT_CMDS) {
        tbOutputData[tbOutputCount][CMD_LEN] = len + CMD_HEADER_LEN;
        tbOutputData[tbOutputCount][CMD_TO] = to;
        tbOutputData[tbOutputCount][CMD_FROM] = from;
        tbOutputData[tbOutputCount][CMD_TYPE] = type;

        if (len > 0) {
            dataToCmd: for (int i = 0; i < len; i++) {
                tbOutputData[tbOutputCount][CMD_HEADER_LEN + i] = data[i];
            }
        }

        tbOutputCount++;
    } else {
        std::cout << "Cannot send message, output buffer full" << std::endl;
    }
}

//============================================================================//
// Debug ---------------------------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Parses a message and prints it out to the standard output.
 *
 * @param   send    True if the message is being sent, false otherwise
 * @param   data    The array containing the message
 *
 * @return  None
 *
 ******************************************************************************/
void tbPrintCommand(bool send, uint32 *data) {
    if (send) {
        if (data[CMD_TO] == CMD_BROADCAST) {
            std::cout << "-> TX, TestBench sent broadcast:                   ";
        } else if (data[CMD_TO] == BOIDGPU_ID) {
            std::cout << "-> TX, TestBench sent command to BoidGPU:          ";
        } else if (data[CMD_TO] == CONTROLLER_ID) {
            std::cout << "-> TX, TestBench sent command to BoidMaster:       ";
        } else {
            std::cout << "-> TX, TestBench sent command to " << data[CMD_TO]
                    << ":               ";
        }
    } else {
        if (data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen - BoidGPU should just receive
            std::cout << "<- RX, TestBench received command from BoidGPU:    ";
        } else if (data[CMD_FROM] == CONTROLLER_ID) {
            std::cout << "<- RX, TestBench received command from BoidMaster: ";
        } else {
            std::cout << "<- RX, TestBench received command from "
                    << data[CMD_FROM] << ":         ";
        }
    }

    switch (data[CMD_TYPE]) {
    case MODE_INIT:
        std::cout << "initialise self                    ";
        break;
    case CMD_PING:
        std::cout << "BoidCPU ping                       ";
        break;
    case CMD_PING_REPLY:
        std::cout << "BoidCPU ping response              ";
        break;
    case CMD_USER_INFO:
        std::cout << "user info                          ";
        break;
    case CMD_SIM_SETUP:
        std::cout << "setup BoidCPU                      ";
        break;
    case MODE_CALC_NBRS:
        std::cout << "calculate neighbours               ";
        break;
    case CMD_NBR_REPLY:
        std::cout << "neighbouring boids from neighbour  ";
        break;
    case MODE_POS_BOIDS:
        std::cout << "calculate new boid positions       ";
        break;
    case MODE_LOAD_BAL:
        std::cout << "load balance mode                  ";
        break;
    case CMD_LOAD_BAL:
        std::cout << "load balance instructions          ";
        break;
    case CMD_LOAD_BAL_REQUEST:
        std::cout << "load balance request               ";
        break;
    case MODE_TRAN_BOIDS:
        std::cout << "transfer boids                     ";
        break;
    case CMD_BOID:
        std::cout << "boid in transit                    ";
        break;
    case MODE_DRAW:
        std::cout << "send boids to BoidGPU              ";
        break;
    case CMD_DRAW_INFO:
        std::cout << "boid info heading to BoidGPU       ";
        break;
    case CMD_ACK:
        std::cout << "ACK signal                         ";
        break;
    case CMD_PING_END:
        std::cout << "end of ping                        ";
        break;
    case CMD_PING_START:
        std::cout << "start of ping                      ";
        break;
    case CMD_KILL:
        std::cout << "kill simulation                    ";
        break;
    default:
        std::cout << "UNKNOWN COMMAND: (" << data[CMD_TYPE] << ")              ";
        break;
    }

    int i = 0;
    for (i = 0; i < CMD_HEADER_LEN; i++) {
        std::cout << data[i] << " ";
    }
    std::cout << "|| ";

    for (i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
        std::cout << data[CMD_HEADER_LEN + i] << " ";
    }
    std::cout << std::endl;
}
