/** 
 * Copyright 2015 abradbury
 * 
 * gatekeeper.c
 * 
 * This file represents a Gatekeeper in the system and is programmed onto the 
 * MicroBlaze processor. A Gatekeeper routes data between internal BoidCPUs, 
 * filters out irrelevant external data and forwards internal data to external 
 * entities as appropriate.
 * 
 * This file would need altering depending on the number of BoidCPUs per FPGA 
 * and if the BoidMaster or BoidGPU was on an FPGA. For more information, see 
 * the 'Gatekeeper Configuration' section of this file below. 
 * 
 * The MULTICAST recipient ID was introduced to reduce the number of messages 
 * being sent in the system. Instead of a BoidCPU sending 8 identical messages 
 * to its 8 neighbouring BoidCPUs, with MULTICAST it need only send 1 message 
 * and Gatekeepers/BoidCPUs would look at this message only if the BoidCPU that 
 * send it was a neighbour. 
 * 
 * This Gatekeeper component was developed using the Xilnix Software 
 * Development Kit (SDK) 14.7 and deployed to multiple Xilinx Spartan-6 LX45 
 * FPGAs using Xilinx Platform Studio (XPS) 14.7.
 *
 * TODO: The Gatekeeper code became quite complicated towards the end of the 
 *  project - try and simplify it. 
 *
 ******************************************************************************/


/******************************** Include Files *******************************/

#include "xparameters.h"    // Xilinx definitions
#include "xemaclite.h"      // Ethernet
#include "xintc.h"          // Interrupt controller
#include "xil_exception.h"  // Supports interrupts
#include "xuartlite_l.h"    // UART
#include "fsl.h"            // AXI Steam

#include <stdlib.h>         // For atoi()
#include <stdint.h>         // For specific data types e.g. int16_t

#include "boids.h"          // Boid definitions

/************************** Gatekeeper Configuration **************************/

/**
 * The setup presented here is for an FPGA containing 1 BoidMaster core and
 * 2 BoidCPU cores with the gatekeeper also acting as the BoidGPU.
 *
 * When changing the number of BoidCPUs that this gatekeeper serves, the user
 * will also manually need to add extra code in the checkForInput(),
 * sendInternalMessage(), getFSLData() and putFSLData(). This is primarily due
 * to the Xilinx putfslx() and getfslx() method not allowing a variable channel
 * name.
 *
 * TODO: Investigate ways of reducing the number of changes that need to be
 * made when changing the number of BoidCPUs e.g. with a configuration file
 */

#define MASTER_IS_RESIDENT      1   // Define when the BoidMaster is resident
#define ACT_AS_BOIDGPU          1   // Define if acting as BoidGPU
#define DEBUG                   1   // Define to print out debug messages

#define RESIDENT_BOIDCPU_COUNT  2   // The number of resident BoidCPUs

#ifdef MASTER_IS_RESIDENT
#define BOIDMASTER_CHANNEL      0
#endif

#define BOIDCPU_CHANNEL_1       1
#define BOIDCPU_CHANNEL_2       2

static u8 own_mac_address[XEL_MAC_ADDR_SIZE] = {0x00, 0x0A, 0x35, 0x01, 0x02, 0x03};
u32 gatekeeperID = 999;

/**************************** Constant Definitions ****************************/

#define BOID_DATA_LENGTH        3
#define EXT_INPUT_SIZE          8   // Number of received external messages to hold

#define ALL_BOIDCPU_CHANNELS    99  // When a message is sent to all channels

#define KILL_KEY                0x6B    // 'k'
#define PAUSE_KEY               0x70    // 'p'

#define EXTERNAL_RECIPIENT              0
#define INTERNAL_RECIPIENT              1
#define INTERNAL_AND_EXTERNAL_RECIPIENT 2

#define EMAC_DEVICE_ID      XPAR_EMACLITE_0_DEVICE_ID
#define INTC_DEVICE_ID      XPAR_INTC_0_DEVICE_ID
#define INTC_EMACLITE_ID    XPAR_INTC_0_EMACLITE_0_VEC_ID

/*************************** Variable Definitions *****************************/

XIntc       intc;               // Interrupt controller
XEmacLite   ether;              // Ethernet

u8 extInputArrivalPtr = 0;      // Next message arrival slot
u8 extInputProcessPtr = 0;      // Next message to process
u8 externalOutput[XEL_HEADER_SIZE + (MAX_CMD_LEN * MAX_OUTPUT_CMDS * 4)];
u8 rawExternalInput[EXT_INPUT_SIZE][XEL_HEADER_SIZE + (MAX_CMD_LEN * MAX_INPUT_CMDS * 4)];
u32 externalInput[MAX_CMD_LEN * MAX_INPUT_CMDS];

// Setup other variables
#ifdef MASTER_IS_RESIDENT
int timeStep = 0;
u32 boidCount = 0;              // Number of simulation boids
u8 channelSetupCounter = 1;
u8 discoveredBoidCPUCount = 0;
static u8 channelIDList[RESIDENT_BOIDCPU_COUNT + 1];
#else
u8 channelSetupCounter = 0;
static u8 channelIDList[RESIDENT_BOIDCPU_COUNT];
#endif

#ifdef ACT_AS_BOIDGPU
int drawnBoidsCount = 0;        // Number of drawn boids
#endif

// If the BoidMaster is present, then it must be on channel 0

u32 messageData[MAX_CMD_BODY_LEN];
bool boidCPUsSetup = false;
bool fowardMessage = true;
bool forwardingInterceptedSetup = false;
u8 ackCount = 0;

u8 residentNbrCounter = 0;
u8 residentBoidCPUNeighbours[MAX_BOIDCPU_NEIGHBOURS * RESIDENT_BOIDCPU_COUNT];

/*************************** Function Prototypes ******************************/
int setupEthernet();

static void EmacLiteRecvHandler(void *CallBackRef);
static void EmacLiteSendHandler(void *CallBackRef);

void registerWithSwitch();
void checkForInput();

void respondToPing();
void interceptSetupInfo(u32 *interceptedData);

u8 internalChannelLookUp(u32 to);
u8 recipientLookUp(u32 to, u32 from);
bool externalMessageRelevant();
void interceptMessage(u32 *data);

void processReceivedExternalMessage();
void processReceivedInternalMessage(u32 *inputData);

void sendMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);

void decodeAndPrintBoids(u32 *data);

#ifdef DEBUG
void printMessage(bool send, u32 *data);
#endif

#ifdef MASTER_IS_RESIDENT
void takeUserInput();
void uiBoidCPUSearch();
void sendKillCommand();
#endif

#ifdef ACT_AS_BOIDGPU
void monitorDrawnBoids(u32* data);
void simulateBoidGPUACK();
#endif

void putFSLData(u32 value, u32 channel);
u32 getFSLData(u32 *data, u32 channel);

void encodeEthernetMessage(u32 outputValue, u8* outputArrayPointer, int* idx);
u32 decodeEthernetMessage(u8 inputZero, u8 inputOne, u8 inputTwo, u8 inputThree);


//============================================================================//
//- Main Method --------------------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * This is the main method and entry point of the Gatekeeper component. 
 * 
 * After setting up the FPGA's Ethernet connection, the Gatekeeper registers 
 * with the Ethernet switch by sending an empty message. The Gatekeeper then 
 * enters an infinite loop where it runs the simulation - dependent on whether 
 * it is a Gatekeeper for the BoidMaster or not. 
 * 
 * If the BoidMaster is resident, the Gatekeeper presents a 'user interface' 
 * over a UART connection to a PC. The user initiates the ping for BoidCPUs and 
 * the Gatekeeper displays any that are found before allowing the user to enter 
 * simulation parameters. Then the Gatekeeper settles in to its main role of 
 * routing traffic. If the user presses the pause or kill key, the simulation 
 * paused or killed, respectively. 
 *
 * If the BoidMaster is not present, the Gatekeeper just listens for input and 
 * routes it as needed. 
 *
 * @param   None
 *
 * @return          0 if success, 1 if failure
 *
 ******************************************************************************/
int main() {
    // Setup own ID - TODO: Ensure that this doesn't clash with the BoidIDs
    // gatekeeperID = *((volatile int *) XPAR_MCB_DDR2_S0_AXI_BASEADDR + 0x01ABCABC);
    // TODO: Assign MAC address and ID randomly, or better

    setupEthernet();
    registerWithSwitch();

    do {
        bool simulationKilled = false;
        do {
            print("------------------------------------------------------\n\r");
            print("---------FPGA Implementation of Flocking Birds--------\n\r");
            print("------------------------------------------------------\n\r");

#ifdef MASTER_IS_RESIDENT
            // Search for BoidCPUs
            uiBoidCPUSearch();

            // Take user input e.g. simulation boid count
            takeUserInput();
#else
            print("Waiting for ping...\n\r");
#endif
            // Finally, begin main loop
            do {
                checkForInput();

#ifdef MASTER_IS_RESIDENT
                // Handle a kill key or a pause key being pressed
                if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
                    int key = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);

                    if (key == KILL_KEY) {
                        simulationKilled = true;
                        sendKillCommand();
                        print("Simulation killed, restarting...\n\r");
                    } else if (key == PAUSE_KEY) {
                        print("Simulation paused, press 'p' to resume\n\r");
                        bool simulationUnpaused = false;
                        do {
                            if (XUartLite_RecvByte(
                                    XPAR_RS232_UART_1_BASEADDR) == PAUSE_KEY) {
                                simulationUnpaused = true;
                            }
                        } while (!simulationUnpaused);
                        print("Simulation resumed\n\r");
                    }
                }
#endif
            } while (!simulationKilled);
        } while (!simulationKilled);
    } while (1);

    return 0;
}

//============================================================================//
//- Message Transfer ---------------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * The main method used to determine if there is any input available on either
 * internal or external lines. If there is, it is processed accordingly.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void checkForInput() {
    // Check for and process received external data ----------------------------
    if (extInputProcessPtr != extInputArrivalPtr) {
#ifdef DEBUG
        print("External messages ready to be processed\n\r");
#endif
        processReceivedExternalMessage();
    }

    // Check for internal (FXL/AXI) data ---------------------------------------
#ifdef MASTER_IS_RESIDENT
    u32 boidMasterData[MAX_CMD_LEN];
    int boidMasterInvalid = getFSLData(boidMasterData, BOIDMASTER_CHANNEL);
#endif

    u32 boidCPUChannelOneData[MAX_CMD_LEN];
    int boidCPUChannelOneInvalid = getFSLData(boidCPUChannelOneData, BOIDCPU_CHANNEL_1);

    u32 boidCPUChannelTwoData[MAX_CMD_LEN];
    int boidCPUChannelTwoInvalid = getFSLData(boidCPUChannelTwoData, BOIDCPU_CHANNEL_2);

    // Process received internal data ------------------------------------------
#ifdef MASTER_IS_RESIDENT
    if (!boidMasterInvalid) {
        processReceivedInternalMessage(boidMasterData);
    }
#endif

    if (!boidCPUChannelOneInvalid) {
        processReceivedInternalMessage(boidCPUChannelOneData);
    }

    if (!boidCPUChannelTwoInvalid) {
        processReceivedInternalMessage(boidCPUChannelTwoData);
    }
}

//============================================================================//
//- Message Reception Functions ----------------------------------------------//
//============================================================================//

#ifdef ACT_AS_BOIDGPU
/******************************************************************************/
/*
 * If the Gatekeeper is acting as the BoidGPU, monitor the data passing through 
 * the Gatekeeper and if it is boids heading to the BoidCPU starting counting 
 * until all the system boids have been received. Then issue an ACK. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void monitorDrawnBoids(u32* data) {
    if (data[CMD_TYPE] == CMD_DRAW_INFO) {
        drawnBoidsCount +=  (data[CMD_LEN] - CMD_HEADER_LEN - 1) / BOID_DATA_LENGTH;

        if (drawnBoidsCount == boidCount) {
            drawnBoidsCount = 0;
            simulateBoidGPUACK();
        }
    }
}

/******************************************************************************/
/*
 * If the Gatekeeper is acting as the BoidGPU, then issue an ACK when all the 
 * boids have been 'drawn'. This signals to the BoidMaster to move the 
 * simulation on to the next time step. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulateBoidGPUACK() {
    xil_printf("TIME STEP: %d\n\r", timeStep);
    timeStep++;

    sendInternalMessage(0, CONTROLLER_ID, BOIDGPU_ID, CMD_ACK, messageData);
}
#endif

/******************************************************************************/
/*
 * Process a message received internally from one of the resident BoidCPUs over 
 * the FLS.AXI connections. The message is then printed, parsed and forwarded 
 * as necessary. 
 * 
 * @param   inputData   The internal message to process
 *
 * @return  None
 *
 ******************************************************************************/
void processReceivedInternalMessage(u32 *inputData) {
#ifdef ACT_AS_BOIDGPU
    monitorDrawnBoids(inputData);
#endif

    fowardMessage = true;
#ifdef DEBUG
    print("INTERNAL: ");
    printMessage(false, inputData);
#endif
#ifdef ACT_AS_BOIDGPU
    decodeAndPrintBoids(inputData);
#endif

    if ((!boidCPUsSetup) && (inputData[CMD_TYPE] != CMD_ACK)) {
        interceptMessage(inputData);
    }

    // Collect the ACKs for the recipient BoidCPUs and issue a collective one
    if (inputData[CMD_TYPE] == CMD_ACK) {
        fowardMessage = false;
        ackCount++;

        if (ackCount == RESIDENT_BOIDCPU_COUNT) {
#ifdef DEBUG
            print ("All ACKs received \n\r");
#endif
            sendMessage(0, CONTROLLER_ID, gatekeeperID, CMD_ACK, messageData);
            ackCount = 0;
        }
#ifdef DEBUG
        else {
            xil_printf("Waiting for ACKs (received %d of %d)...\n\r", ackCount,
                    RESIDENT_BOIDCPU_COUNT);
        }
#endif
    }
#ifdef ACT_AS_BOIDGPU
    else if (inputData[CMD_TO] == BOIDGPU_ID) {
        fowardMessage = false;
    }
#endif

    // Forward the message, if needed
    if (fowardMessage) {
        u32 inputDataBodyLength = inputData[CMD_LEN] - CMD_HEADER_LEN;
        u32 inputDataBody[inputDataBodyLength];
        int i = 0;
        for (i = 0; i < inputDataBodyLength; i++) {
            inputDataBody[i] = inputData[CMD_HEADER_LEN + i];
        }

        sendMessage(inputDataBodyLength, inputData[CMD_TO],
                inputData[CMD_FROM], inputData[CMD_TYPE], inputDataBody);
    }
}

/******************************************************************************/
/*
 * Process a message received from another FPGA over the Ethernet connection. 
 * Messages received over this connection need converting from a 8-bit 
 * representation to a 32-bit representation, which is used internally. The 
 * message is then printed, parsed and forwarded as necessary. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void processReceivedExternalMessage() {
    // Move the message and strip the header
    int j = 0, k = 0;
    for (j = XEL_HEADER_SIZE, k = 0; j < (MAX_CMD_LEN * MAX_INPUT_CMDS); j+=4, k++) {
        externalInput[k] = decodeEthernetMessage(
                rawExternalInput[extInputProcessPtr][j + 0],
                rawExternalInput[extInputProcessPtr][j + 1],
                rawExternalInput[extInputProcessPtr][j + 2],
                rawExternalInput[extInputProcessPtr][j + 3]);
    }

    XEmacLite_FlushReceive(&ether); // Clear any received messages

#ifdef ACT_AS_BOIDGPU
    monitorDrawnBoids(externalInput);
#endif

    if (!boidCPUsSetup) {
#ifdef DEBUG
        print("BoidCPUs not setup\n\r");
#endif

        // Then process the remaining information
#ifdef DEBUG
        print("EXTERNAL: ");
        printMessage(false, externalInput);
#endif
#ifdef ACT_AS_BOIDGPU
        decodeAndPrintBoids(externalInput);
#endif

        interceptMessage(externalInput);
    } else if (externalMessageRelevant()) {
        // Then process the remaining information
#ifdef DEBUG
        print("EXTERNAL: ");
        printMessage(false, externalInput);
#endif
#ifdef ACT_AS_BOIDGPU
        decodeAndPrintBoids(externalInput);
#endif

        // Forward the message
        u32 dataBodyLength = externalInput[CMD_LEN] - CMD_HEADER_LEN;
        u32 dataBody[dataBodyLength];
        int i = 0;
        for (i = 0; i < dataBodyLength; i++) {
            dataBody[i] = externalInput[CMD_HEADER_LEN + i];
        }

        sendMessage(dataBodyLength, externalInput[CMD_TO],
                externalInput[CMD_FROM], externalInput[CMD_TYPE], dataBody);
    }


    extInputProcessPtr = (extInputProcessPtr + 1) % EXT_INPUT_SIZE;
}

//============================================================================//
//- Message Transmission Functions -------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Send a message - determines whether the message should be sent internally, 
 * externally or both and then sends the message as appropriate. 
 *
 * @param   len     The length of the message body
 * @param   to      The ID of the recipient of the message
 * @param   from    The ID of the message sender
 * @param   type    The type of the message (defined in boids.h)
 * @param   data    The message data
 *
 * @return  None
 *
 ******************************************************************************/
void sendMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
    u8 recipientLocation;

    // If the BoidCPUs are not yet set up, a setup message is being sent and
    // the message is addressed to this Gatekeeper, send internally. If it is
    // not addressed to this Gatekeeper, send it externally.
    if ((!boidCPUsSetup) && (type == CMD_SIM_SETUP)) {
#ifdef DEBUG
        print("BoidCPUs not setup and setup message being sent...\n\r");
#endif

        if ((to == gatekeeperID) || (forwardingInterceptedSetup)) {
            recipientLocation = INTERNAL_RECIPIENT;
        } else {
            recipientLocation = EXTERNAL_RECIPIENT;
        }
    }
#ifdef MASTER_IS_RESIDENT
    // Don't forward a ping to internal BoidCPUs
    else if (type == CMD_PING) {
        recipientLocation = EXTERNAL_RECIPIENT;
    }
#endif
    else {
        recipientLocation = recipientLookUp(to, from);
    }

    // Send a message internally or externally, depending on the recipient
    if (recipientLocation == EXTERNAL_RECIPIENT) {
        sendExternalMessage(len, to, from, type, data);
    } else if (recipientLocation == INTERNAL_RECIPIENT) {
        sendInternalMessage(len, to, from, type, data);
    } else if (recipientLocation == INTERNAL_AND_EXTERNAL_RECIPIENT) {
        sendExternalMessage(len, to, from, type, data);
        sendInternalMessage(len, to, from, type, data);
    }
}

/******************************************************************************/
/*
 * Sends a message internally, over the FSL/AXI bus/stream, to BoidCPUs that
 * reside on the same FPGA that the MicroBlaze does.
 *
 * @param   len     The length of the message body
 * @param   to      The ID of the recipient of the message
 * @param   from    The ID of the message sender
 * @param   type    The type of the message (defined in boids.h)
 * @param   data    The message data
 *
 * @return  None
 *
 ******************************************************************************/
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
    u8 channel;

    // If the BoidCPUs are not yet setup and a setup message is being sent,
    // send the message down a specific channel
    if ((!boidCPUsSetup) && (type == CMD_SIM_SETUP)) {
        channel = channelSetupCounter;
    } else {
        channel = internalChannelLookUp(to);
    }

    // First, create the message
    u32 command[MAX_CMD_LEN];
    int i = 0, j = 0;

    command[CMD_LEN] = len + CMD_HEADER_LEN;
    command[CMD_TO] = to;
    command[CMD_FROM] = from;
    command[CMD_TYPE] = type;

    if (len > 0) {
        for (i = 0; i < len; i++) {
            command[CMD_HEADER_LEN + i] = data[i];
        }
    }

    // Multicast messages - don't send back to self
    // For each channel, if the multicast message is not from that channel then
    // send the message down that channel.
    if (to == CMD_MULTICAST) {
#ifdef MASTER_IS_RESIDENT
        for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
        for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
            if (channelIDList[i] != from) {
#ifdef DEBUG
                print("INTERNAL: ");
                printMessage(true, command);
#endif
                for (j = 0; j < CMD_HEADER_LEN + len; j++) {
                    putFSLData(command[j], i);
                }
            }
        }
    } else {
#ifdef DEBUG
        print("INTERNAL: ");
        printMessage(true, command);
#endif

        // Finally, send the message
        for (i = 0; i < CMD_HEADER_LEN + len; i++) {
            switch (channel) {
#ifdef MASTER_IS_RESIDENT
            case BOIDMASTER_CHANNEL:
                putFSLData(command[i], BOIDMASTER_CHANNEL);
                break;
#endif
            case BOIDCPU_CHANNEL_1:
                putFSLData(command[i], BOIDCPU_CHANNEL_1);
                break;
            case BOIDCPU_CHANNEL_2:
                putFSLData(command[i], BOIDCPU_CHANNEL_2);
                break;
            default:
                // Otherwise, send to all BoidCPU channels
                putFSLData(command[i], BOIDCPU_CHANNEL_1);
                putFSLData(command[i], BOIDCPU_CHANNEL_2);
                break;
            }
        }
    }
}

/******************************************************************************/
/*
 * Sends a message to an external entity beyond the FPGA that the MicroBlaze
 * resides on. Ethernet is used to transmit the messages to other FPGAs.
 *
 * @param   len     The length of the message body
 * @param   to      The ID of the recipient of the message
 * @param   from    The ID of the message sender
 * @param   type    The type of the message (defined in boids.h)
 * @param   data    The message data
 *
 * @return  None
 *
 ******************************************************************************/
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
    // First, create the Ethernet message header
    // The destination MAC address, broadcast here
    u8 *buffer = externalOutput;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;

    // The source MAC address
    int i = 0;
    for (i = 0; i < XEL_MAC_ADDR_SIZE; i++) {
        *buffer++ = own_mac_address[i];
    }

    // Add the EtherType field
    *buffer++ = 0x55;
    *buffer++ = 0xAA;

    // The internal messages are 32 bits in size, whereas Ethernet is 8 bits.
    // A simple solution is to split the messages up on sending into 8 bit
    // pieces and join back together on receiving.
    int index = 14;
    encodeEthernetMessage((len + CMD_HEADER_LEN), externalOutput, &index);
    encodeEthernetMessage(to, externalOutput, &index);
    encodeEthernetMessage(from, externalOutput, &index);
    encodeEthernetMessage(type, externalOutput, &index);

    if (len > 0) {
        for (i = 0; i < len; i++) {
            encodeEthernetMessage(data[i], externalOutput, &index);
        }
    }

    const int minEthernetMsgSize = 64;
    int paddedBytes = minEthernetMsgSize - (((len + CMD_HEADER_LEN) * 4) + XEL_HEADER_SIZE);
    int extraByteCounter = 0;
    for (extraByteCounter = 0; extraByteCounter < paddedBytes; extraByteCounter++) {
        externalOutput[index + extraByteCounter] = 0;
    }

    // Then create the data to send, create the message
#ifdef DEBUG
    u32 command[MAX_CMD_LEN];

    command[CMD_LEN] = len + CMD_HEADER_LEN;
    command[CMD_TO] = to;
    command[CMD_FROM] = from;
    command[CMD_TYPE] = type;

    if (len > 0) {
        for (i = 0; i < len; i++) {
            command[CMD_HEADER_LEN + i] = data[i];
        }
    }

    print("EXTERNAL: ");
    printMessage(true, command);
#endif

    // Finally, clear the receive buffer before sending
    // XEmacLite_FlushReceive(&ether);
    int status = XEmacLite_Send(&ether, externalOutput, extraByteCounter + XEL_HEADER_SIZE + ((len + CMD_HEADER_LEN) * 4));

#ifdef DEBUG
    if (status == 1) {
        print("**** Failed to send external message\n\r");
    } else {
        print("External message sent successfully \n\r");
    }
#endif
}

//============================================================================//
//- Message Transceive Supporting Functions ----------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Determine if a received external message should be forwarded internally or
 * ignored. It should be forwarded internally if:
 *  - it is from the BoidMaster
 *  - it is a broadcast command
 *  - it is from a BoidCPU that is a neighbour of a resident BoidCPU
 *  - it is addressed to the BoidMaster AND the BoidMaster is resident
 *
 * @param   None
 *
 * @return          True if the message should be forwarded internally
 *
 ******************************************************************************/
bool externalMessageRelevant() {
    bool result = false;

    if (externalInput[CMD_FROM] == CONTROLLER_ID) {
        result = true;
    } else if (externalInput[CMD_TO] == CMD_BROADCAST) {
        result = true;
    } else if (externalInput[CMD_TO] == BOIDGPU_ID) {
        result = false;
#ifdef ACT_AS_BOIDGPU
        decodeAndPrintBoids(externalInput);
#endif
    }

#ifdef MASTER_IS_RESIDENT
    else if (externalInput[CMD_TO] == CONTROLLER_ID) {
        result = true;
    }
#endif

    else if (externalInput[CMD_FROM] >= FIRST_BOIDCPU_ID) {
        int i = 0;
        for (i = 0; i < residentNbrCounter; i++) {
            if (externalInput[CMD_FROM] == residentBoidCPUNeighbours[i]) {
                result = true;
            }
        }
    }

    return result;
}

/******************************************************************************/
/*
 * Determine whether the message recipient is internal or external to the
 * Gatekeeper, or whether the message needs to be sent both internally and
 * externally. 
 *
 * @param   to      The recipient of the message
 * @param   from    The sender of the message
 *
 * @return          0 if external, 1 if internal, 2 if both
 *
 ******************************************************************************/
u8 recipientLookUp(u32 to, u32 from) {
    u8 recipientInterface;
    bool intAndExt = false;
    int i = 0;

    switch (to) {
        case CMD_BROADCAST:
#ifdef MASTER_IS_RESIDENT
            recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
#else
            recipientInterface = INTERNAL_RECIPIENT;
#endif
            break;
        case CONTROLLER_ID:
#ifdef MASTER_IS_RESIDENT
            recipientInterface = INTERNAL_RECIPIENT;
#else
            recipientInterface = EXTERNAL_RECIPIENT;
#endif
            break;
        case BOIDGPU_ID:
            recipientInterface = EXTERNAL_RECIPIENT;
            break;
        case CMD_MULTICAST:
            // If from resident, INTERNAL_AND_EXTERNAL_RECIPIENT;
            // Else INTERNAL_RECIPIENT;

#ifdef MASTER_IS_RESIDENT
            for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
            for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
                if (channelIDList[i] == from) {
                    recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
                    intAndExt = true;
                    break;
                }
                if (!intAndExt) recipientInterface = INTERNAL_RECIPIENT;
            }
            break;
        default:
            if (to >= FIRST_BOIDCPU_ID) {
                bool internal = false;
                int i = 0;
#ifdef MASTER_IS_RESIDENT
                for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
                for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
                    if (channelIDList[i] == to) {
                        recipientInterface = INTERNAL_RECIPIENT;
                        internal = true;
                        break;
                    }
                }
                if(!internal) recipientInterface = EXTERNAL_RECIPIENT;

            } else {
                recipientInterface = INTERNAL_AND_EXTERNAL_RECIPIENT;
            }

            break;
    }

    return recipientInterface;
}

//============================================================================//
//- Message Interception Functions -------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * If the BoidCPUs are not yet setup, intercept certain messages. A ping
 * message is intercepted and the Gatekeeper responds on behalf of the
 * BoidCPUs. The BoidCPU setup information is intercepted in order for the
 * Gatekeeper to know the IDs of its resident BoidCPUs. A ping reply is
 * intercepted, if the BoidMaster is present, and outputted to the UI.
 *
 * @param   interceptedData     The intercepted message/command
 *
 * @return  None
 *
 ******************************************************************************/
void interceptMessage(u32 *interceptedData) {
    if (interceptedData[CMD_TYPE] == CMD_PING) {
        respondToPing();
    } else if ((interceptedData[CMD_TO] == gatekeeperID)
            && (interceptedData[CMD_TYPE] == CMD_SIM_SETUP)) {
        fowardMessage = false;
        forwardingInterceptedSetup = true;
        interceptSetupInfo(interceptedData);
        forwardingInterceptedSetup = false;
    }

#ifdef MASTER_IS_RESIDENT
    // This should only ever be received from an external source
    else if (interceptedData[CMD_TYPE] == CMD_PING_REPLY) {
        xil_printf("found %d BoidCPU(s)..", interceptedData[CMD_HEADER_LEN]);
        discoveredBoidCPUCount += interceptedData[CMD_HEADER_LEN];
        xil_printf("total (%d)..\n\r", discoveredBoidCPUCount);

        // Forward the data
        int i = 0;
        u32 dataBodyLength = interceptedData[CMD_LEN] - CMD_HEADER_LEN;
        u32 dataBody[dataBodyLength];
        for (i = 0; i < dataBodyLength; i++) {
            dataBody[i] = interceptedData[CMD_HEADER_LEN + i];
        }

        sendMessage(dataBodyLength, interceptedData[CMD_TO],
                interceptedData[CMD_FROM], interceptedData[CMD_TYPE], dataBody);
    }
#endif
}

/******************************************************************************/
/*
 * When the Gatekeeper detects a ping message, it responds on behalf of the
 * resident BoidCPUs with the number of resident BoidCPUs. Because Gatekeepers 
 * have to be hardcoded with the number of FSL/AXI channels for the number of 
 * BoidCPUs, they know how many BoidCPUs they manage, so can reply on behalf of 
 * the BoidCPUs. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void respondToPing() {
#ifdef DEBUG
    print("Gatekeeper generating ping response...\n\r");
#endif

    messageData[0] = RESIDENT_BOIDCPU_COUNT;
    sendMessage(1, CONTROLLER_ID, gatekeeperID, CMD_PING_REPLY, messageData);

#ifdef MASTER_IS_RESIDENT
    xil_printf("found %d BoidCPU(s) - ", RESIDENT_BOIDCPU_COUNT);
    discoveredBoidCPUCount += RESIDENT_BOIDCPU_COUNT;
    xil_printf("total (%d)..\n\r", discoveredBoidCPUCount);
#endif
}

/******************************************************************************/
/*
 * When the Gatekeeper detects a setup command addressed to it, it extracts
 * the newly-assigned BoidCPU IDs and BoidCPU neighbours before forwarding the
 * message to one of the resident BoidCPUs. This is used so that the Gatekeeper 
 * knows how to route messages. 
 *
 * @param   setupData   The setup data bound for a resident BoidCPU
 *
 * @return  None
 *
 ******************************************************************************/
void interceptSetupInfo(u32 * setupData) {
#ifdef DEBUG
    print("Gatekeeper intercepted setup data...\n\r");
#endif

    // Store the resident BoidCPU IDs and associated channel
    channelIDList[channelSetupCounter] = (u8)setupData[CMD_HEADER_LEN
            + CMD_SETUP_NEWID_IDX];

    // Update Gatekeeper's neighbour list
    int i = 0, j = 0;
    for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        u8 nbr = setupData[CMD_HEADER_LEN + CMD_SETUP_BNBRS_IDX + i];
        bool neighbourAlreadyListed = false;

        for (j = 0; j < residentNbrCounter; j++) {
            if (nbr == residentBoidCPUNeighbours[j]) {
                neighbourAlreadyListed = true;
                break;
            }
        }

        if (!neighbourAlreadyListed) {
            residentBoidCPUNeighbours[residentNbrCounter] = nbr;
            residentNbrCounter++;
        }
    }

    // Forward the data
    u32 setupDataBodyLength = setupData[CMD_LEN] - CMD_HEADER_LEN;
    u32 setupDataBody[setupDataBodyLength];
    for (i = 0; i < setupDataBodyLength; i++) {
        setupDataBody[i] = setupData[CMD_HEADER_LEN + i];
    }

    sendMessage(setupDataBodyLength, CMD_BROADCAST, setupData[CMD_FROM],
            setupData[CMD_TYPE], setupDataBody);

    // Update counters
    channelSetupCounter++;
#ifdef MASTER_IS_RESIDENT
    if (channelSetupCounter == (RESIDENT_BOIDCPU_COUNT + 1)) {
#else
        if (channelSetupCounter == RESIDENT_BOIDCPU_COUNT) {
#endif
        boidCPUsSetup = true;
#ifdef DEBUG
        print("BoidCPUs now set up\n\r");
#endif
    }
}

//============================================================================//
//- BoidMaster Support -------------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Sends CMD_KILL to all entities in the system, on behalf of the BoidMaster.
 * Called when the user pressed the KILL_KEY. Not fully implemented.
 *
 * TODO: Need to reset a load of variables...
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void sendKillCommand() {
    sendExternalMessage(0, CMD_BROADCAST, gatekeeperID, CMD_KILL, messageData);
    sendInternalMessage(0, CMD_BROADCAST, gatekeeperID, CMD_KILL, messageData);
}

#ifdef MASTER_IS_RESIDENT
/******************************************************************************/
/*
 * Handles the process of taking user input for the simulation setup values. 
 * Currently, on the number of boids in the system can be inputted. 
 *
 * TODO: Allow more simulation parameters to be inputted by the user
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void takeUserInput() {
    bool boidCountValid = false;    // True if count is valid
    u8 index = 0;                   // Index for keyPresses
    char keyPress;                  // The key pressed
    char keyPresses[] = "";         // An array of the keys pressed

    while (!boidCountValid) {
        print("Enter boid count: ");

        do {
            keyPress = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);
//          xil_printf("%d entered\n\r", keyPress);

            if (!USING_VLAB) {
                XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);
            }

            keyPresses[index] = keyPress;
            index++;
        } while (keyPress != WINDOWS_ENTER_KEY);

        // Check if entered boid count is valid
        boidCount = (u32) atoi(keyPresses);

        if (boidCount > 0) {
            print("\n\r");
            boidCountValid = true;
        } else {
            print("\n\r**Error: boid count must be greater than 0."
                    " Please try again.\n\r");
        }
    }

    // If the user-entered data is valid, sent it to the controller
    messageData[0] = boidCount;
    sendInternalMessage(1, CONTROLLER_ID, gatekeeperID, CMD_USER_INFO,
            messageData);
}

/******************************************************************************/
/*
 * Handles the process of searching for BoidCPUs and displaying it on the UI. 
 * The search is ended when the user presses the enter key. If no BoidCPUs have 
 * been found in this time, the search resumes. As BoidCPUs are found, a 
 * message is printed to the UI informing the user.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void uiBoidCPUSearch() {
    bool boidCPUSearchComplete = false;
    do {
        print("Searching for BoidCPUs (press ENTER to stop)...\n\r");
        sendInternalMessage(0, CONTROLLER_ID, gatekeeperID, CMD_PING_START,
                messageData);

        bool enterKeyPressed = false;
        do {
            checkForInput();

            // If the ENTER key is pressed, exit search
            if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
                int key = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);

                if ((key == LINUX_ENTER_KEY) || (key == WINDOWS_ENTER_KEY)) {
                    enterKeyPressed = true;
                }
            }
        } while (!enterKeyPressed);

        // If BoidCPUs have been found, exit search, else restart
        if (discoveredBoidCPUCount > 0) {
            boidCPUSearchComplete = true;
            xil_printf("\n\r%d BoidCPUs found\n\r", discoveredBoidCPUCount);
        } else {
            print("\n\rNo BoidCPUs found, trying again...\n\r");
        }
    } while (!boidCPUSearchComplete);

    // When the ping search is complete, inform the BoidMaster
    sendInternalMessage(0, CONTROLLER_ID, gatekeeperID, CMD_PING_END,
            messageData);
}

#endif

/******************************************************************************/
/*
 * Each gatekeeper/MicroBlaze needs to send a broadcast message so that the
 * Ethernet switch can build its address table. Need a delay after sending, so
 * get the user to press enter to begin the setup process.
 * 
 * TODO: Perhaps the construction and sending of the registration message 
 *      should be separated from the printout and waiting for the user input?
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void registerWithSwitch() {
    print("Registering with the switch\n\r");

    // Register with the switch
    u8 *buffer = externalOutput;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;
    *buffer++ = 0xFF;

    // Write the source MAC address
    int m = 0;
    for (m = 0; m < 6; m++) {
        *buffer++ = own_mac_address[m];
    }

    // Write the type/length field
    *buffer++ = 0x55;
    *buffer++ = 0xAA;

    XEmacLite_Send(&ether, externalOutput, XEL_HEADER_SIZE);

#ifdef MASTER_IS_RESIDENT
    print("------------------------------------------------------\n\r");
    print("---------FPGA Implementation of Flocking Birds--------\n\r");
    print("------------------------------------------------------\n\r");
    print("Please press ENTER to begin the simulation setup\n\r");

    // This is needed to give the switch time to process the register message
    bool enterKeyPressed = false;
    do {
        // If the ENTER key is pressed, exit search
        if (!XUartLite_IsReceiveEmpty(XPAR_RS232_UART_1_BASEADDR)) {
            int key = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);

            if ((key == LINUX_ENTER_KEY) || (key == WINDOWS_ENTER_KEY)) {
                enterKeyPressed = true;
            }
        }
    } while (!enterKeyPressed);
#endif
}

//============================================================================//
//- Supporting Functions -----------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * Determines the channel that at message should be sent down to reach the
 * recipient BoidCPU. If a suitable channel cannot be found, it is sent to all.
 *
 * @param   to      The ID of the message recipient
 *
 * @return          The FSL channel to send the message down, 99 if all channels
 *
 ******************************************************************************/
u8 internalChannelLookUp(u32 to) {
    int i = 0;

#ifdef MASTER_IS_RESIDENT
    if (to == CONTROLLER_ID) {
        return BOIDMASTER_CHANNEL;
    }

    for (i = 1; i < (RESIDENT_BOIDCPU_COUNT + 1); i++) {
#else
        for (i = 0; i < RESIDENT_BOIDCPU_COUNT; i++) {
#endif
        if (channelIDList[i] == to) {
            return i;
        }
    }

    return ALL_BOIDCPU_CHANNELS;    // e.g. on broadcast
}

/******************************************************************************/
/*
 * A wrapper for the getfsxl() method. Checks if there is any data on the
 * specified channel and reads in the entire message/command if there is. 
 * Notifies of errors. 
 *
 * @param   data        A pointer to an array to store the read data to
 * @param   channel     The FSL channel to read from
 *
 * @return              0 if the read data was valid, 1 otherwise
 *
 ******************************************************************************/
u32 getFSLData(u32 *data, u32 channel) {
    int invalid, error, value = 0;

    if (channel == BOIDCPU_CHANNEL_1)
        getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
    else if (channel == BOIDCPU_CHANNEL_2)
        getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
#ifdef MASTER_IS_RESIDENT
    else if (channel == BOIDMASTER_CHANNEL)
        getfslx(value, BOIDMASTER_CHANNEL, FSL_NONBLOCKING);
#endif

    fsl_isinvalid(invalid);             // Was there any data?
    fsl_iserror(error);                 // Was there an error?

    if (error) {
        xil_printf("Error receiving data on Channel %d: %d\n\r", channel,
                error);
    }

    if (!invalid) {
        data[CMD_LEN] = value;
//      xil_printf("Received data (Channel %d)\n\r", channel);
        int i = 0;

        // Handle invalid length values
        if ((data[CMD_LEN] == 0) || (data[CMD_LEN] > MAX_CMD_LEN)) {
            data[CMD_LEN] = MAX_CMD_LEN;
            print("Message has invalid length - correcting\n\r");
        }

        for (i = 0; i < data[CMD_LEN] - 1; i++) {
            if (channel == BOIDCPU_CHANNEL_1)
                getfslx(value, BOIDCPU_CHANNEL_1, FSL_NONBLOCKING);
            else if (channel == BOIDCPU_CHANNEL_2)
                getfslx(value, BOIDCPU_CHANNEL_2, FSL_NONBLOCKING);
#ifdef MASTER_IS_RESIDENT
            else if (channel == BOIDMASTER_CHANNEL)
                getfslx(value, BOIDMASTER_CHANNEL, FSL_NONBLOCKING);
#endif

            fsl_iserror(error);             // Was there an error?
            if (error) {
                xil_printf("Error receiving data on Channel %d: %d\n\r",
                        channel, error);
            }

            data[i + 1] = value;
        }
    }

    return invalid;
}

/******************************************************************************/
/*
 * A wrapper for putting data onto the (internal) FSL bus. Can catch write
 * errors and alert when the buffer is full.
 *
 * @param   value       The value to place onto the bus
 * @param   channel     The FSL channel to write to
 *
 * @return  None
 *
 ******************************************************************************/
void putFSLData(u32 value, u32 channel) {
    int error = 0, invalid = 0;

    if (channel == BOIDCPU_CHANNEL_1)
        putfslx(value, BOIDCPU_CHANNEL_1, FSL_DEFAULT);
    else if (channel == BOIDCPU_CHANNEL_2)
        putfslx(value, BOIDCPU_CHANNEL_2, FSL_DEFAULT);
#ifdef MASTER_IS_RESIDENT
    else if (channel == BOIDMASTER_CHANNEL)
        putfslx(value, BOIDMASTER_CHANNEL, FSL_DEFAULT);
#endif

    fsl_isinvalid(invalid);
    fsl_iserror(error);

    if (invalid) {
        xil_printf("Warning - channel %d is full: %d\n\r", channel, value);
    }

    if (error) {
        xil_printf("Error writing data to channel %d: %d\n\r", channel, value);
    }
}

/******************************************************************************/
/*
 * Splits a 32-bit value into four 8-bit values for transmission over Ethernet.
 *
 * @param   outputValue         The 32-bit value to split into 4 8-bit values
 * @param   outputArrayPointer  A pointer to the array to store the results
 * @param   idx                 The index location in the array to store at
 *
 * @return  None
 *
 ******************************************************************************/
void encodeEthernetMessage(u32 outputValue, u8* outputArrayPointer, int* idx) {
    outputArrayPointer[*idx + 0] = (outputValue & 0xff000000UL) >> 24;
    outputArrayPointer[*idx + 1] = (outputValue & 0x00ff0000UL) >> 16;
    outputArrayPointer[*idx + 2] = (outputValue & 0x0000ff00UL) >>  8;
    outputArrayPointer[*idx + 3] = (outputValue & 0x000000ffUL);

    *idx += 4;
}

/******************************************************************************/
/*
 * Take four 8 bit values from Ethernet and combine into one 32 bit value
 * 
 * @param   inputZero   The most significant 8-bit byte
 * @param   inputOne    The second most significant 8-bit byte
 * @param   inputTwo    The second least significant 8-bit byte
 * @param   inputThree  The least significant 8-bit byte
 *
 * @return              An unsigned 32-bit integer of the combined input bytes
 *
 ******************************************************************************/
u32 decodeEthernetMessage(u8 inputZero, u8 inputOne, u8 inputTwo, u8 inputThree) {
    return (u32)((inputZero << 24) | (inputOne << 16) | (inputTwo << 8) | (inputThree));
}

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
#ifdef DEBUG
void printMessage(bool send, u32 *data) {
    bool drawnAlready = false;
    bool unknownMessage = false;

    if (send) {
        if (data[CMD_TO] == CONTROLLER_ID) {
            print("-> TX, Gatekeeper sent command to BoidMaster:       ");
        } else if (data[CMD_TO] == CMD_BROADCAST) {
            print("-> TX, Gatekeeper sent broadcast:                   ");
        } else if (data[CMD_TO] == BOIDGPU_ID) {
            print("-> TX, Gatekeeper sent command to BoidGPU:          ");
        } else if (data[CMD_TO] == CMD_MULTICAST) {
            print("-> TX, Gatekeeper sent command to MULTICAST:        ");
        } else {
            xil_printf("-> TX, Gatekeeper sent command to %d:                ",
                    data[CMD_TO]);
        }
    } else {
        if (data[CMD_FROM] == CONTROLLER_ID) {
            print("-----------------------------------------------------------"
                    "---------------------------------------------------\n\r");
            print("<- RX, Gatekeeper received command from BoidMaster: ");
        } else if (data[CMD_TO] == CMD_BROADCAST) {
            // This should never happen - BoidCPUs not be able to broadcast
            xil_printf("<- RX, Gatekeeper received broadcast from %d:       ",
                    data[CMD_FROM]);
        } else if (data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen
            print("<- RX, Gatekeeper received command from BoidGPU:    ");
        } else {
            xil_printf("<- RX, Gatekeeper received command from %d:          ",
                    data[CMD_FROM]);
        }
    }

    switch (data[CMD_TYPE]) {
    case MODE_INIT:
        print("initialise self                   ");
        break;
    case CMD_PING:
        print("BoidCPU ping                      ");
        break;
    case CMD_PING_REPLY:
        print("BoidCPU ping response             ");
        break;
    case CMD_USER_INFO:
        print("output user info                  ");
        break;
    case CMD_SIM_SETUP:
        print("setup BoidCPU                     ");
        break;
    case MODE_CALC_NBRS:
        print("calculate neighbours              ");
        break;
    case CMD_NBR_REPLY:
        print("neighbouring boids from neighbour ");
        decodeAndPrintBoids(data);
        drawnAlready = true;
        break;
    case MODE_POS_BOIDS:
        print("calculate new boid positions      ");
        break;
    case CMD_LOAD_BAL:
        print("load balance                      ");
        break;
    case MODE_TRAN_BOIDS:
        print("transfer boids                    ");
        break;
    case CMD_BOID:
        print("boid in transit                   ");
        break;
    case MODE_DRAW:
        print("send boids to BoidGPU             ");
        break;
    case CMD_DRAW_INFO:
        print("boid info heading to BoidGPU      ");
        decodeAndPrintBoids(data);
        drawnAlready = true;
        break;
    case CMD_ACK:
        print("ACK signal                        ");
        break;
    case CMD_PING_END:
        print("end of ping                       ");
        break;
    case CMD_PING_START:
        print("start of ping                     ");
        break;
    case CMD_KILL:
        print("kill simulation                   ");
        break;
    case CMD_DEBUG:
        print("debug information                 ");
        break;
    default:
        print("UNKNOWN COMMAND                   ");
        unknownMessage = true;
        break;
    }

    if (!drawnAlready) {
        int i = 0;
        for (i = 0; i < CMD_HEADER_LEN; i++) {
            xil_printf("%d ", data[i]);
        }
        print("|| ");

        int range = 0;
        if (!unknownMessage) {
            range = data[CMD_LEN] - CMD_HEADER_LEN;
        } else {
            range = MAX_CMD_LEN;
        }

        for (i = 0; i < range; i++) {
            xil_printf("%d ", data[CMD_HEADER_LEN + i]);
        }
        print("\n\r");
    }
}
#endif

/******************************************************************************/
/*
 * Decodes messages that contain encoded boids and prints out the data. Note
 * that this simply displays the integer part of the data, the fractional part
 * is ignored for simplicity. 
 * 
 * @param   data    The boidal data to decode and print out
 *
 * @return  None
 *
 ******************************************************************************/
void decodeAndPrintBoids(u32 *data) {
#ifndef DEBUG
    if ((data[CMD_TYPE] == CMD_DRAW_INFO)) {
#else
    if ((data[CMD_TYPE] == CMD_DRAW_INFO) || (data[CMD_TYPE] == CMD_NBR_REPLY)) {
#endif
        xil_printf("BoidCPU #%d - ", data[CMD_FROM]);
        int count = (data[CMD_LEN] - CMD_HEADER_LEN - 1) / BOID_DATA_LENGTH;

        int i = 0;
        for (i = 0; i < count; i++) {
            u32 position = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 1];
            u32 velocity = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 2];
            u32 boidID = data[CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 3];

            int16_t xPos = ((int32_t)((int32_t)position >> 16)) >> 4;
            int16_t yPos = ((int32_t)((int16_t)position)) >> 4;

            int16_t xVel = ((int32_t)((int32_t)velocity >> 16)) >> 4;
            int16_t yVel = ((int32_t)((int16_t)velocity)) >> 4;

            xil_printf("#%d: %d %d, %d %d | ", boidID, xPos, yPos, xVel, yVel);
        }
        print("\n\r");
    }
}

/******************************************************************************/
/*
 * Setup the FPGA's Ethernet component (EmacLite). Interrupts were found to be 
 * needed as the Gatekeepers were dropping packets using a polling-based 
 * approach. For further details, see the inline comments.
 * 
 * @param   None
 *
 * @return          0 if success, 1 otherwise - TODO: check these
 *
 ******************************************************************************/
int setupEthernet() {
    int Status;
    XEmacLite_Config *ConfigPtr;

    // Initialise the EmacLite device.
    ConfigPtr = XEmacLite_LookupConfig(EMAC_DEVICE_ID);
    if (ConfigPtr == NULL) {
        return XST_FAILURE;
    }

    Status = XEmacLite_CfgInitialize(&ether, ConfigPtr, ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Set the MAC address.
    XEmacLite_SetMacAddress(&ether, own_mac_address);

    // Empty any existing receive frames.
    XEmacLite_FlushReceive(&ether);

    // Check if there is a Tx buffer available, if there isn't it is an error.
    if (XEmacLite_TxBufferAvailable(&ether) != TRUE) {
        return XST_FAILURE;
    }

    // Set up the interrupt infrastructure.
    // Initialise the interrupt controller driver so that it is ready to use.
    Status = XIntc_Initialize(&intc, INTC_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /*
     * Connect a device driver handler that will be called when an interrupt
     * for the device occurs, the device driver handler performs the
     * specific interrupt processing for the device.
     */
    Status = XIntc_Connect(&intc, INTC_EMACLITE_ID,
        XEmacLite_InterruptHandler, (void *)(&ether));
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /*
     * Start the interrupt controller such that interrupts are enabled for
     * all devices that cause interrupts, specify real mode so that
     * the EmacLite can cause interrupts through the interrupt controller.
     */
    Status = XIntc_Start(&intc, XIN_REAL_MODE);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Enable the interrupt for the EmacLite in the Interrupt controller.
    XIntc_Enable(&intc, INTC_EMACLITE_ID);

    // Initialise the exception table.
    Xil_ExceptionInit();

    // Register the interrupt controller handler with the exception table.
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                (Xil_ExceptionHandler) XIntc_InterruptHandler,
                &intc);

    // Enable non-critical exceptions.
    Xil_ExceptionEnable();

    // Setup the EmacLite handlers.
    XEmacLite_SetRecvHandler((&ether), (void *)(&ether),
                 (XEmacLite_Handler)EmacLiteRecvHandler);
    XEmacLite_SetSendHandler((&ether), (void *)(&ether),
                 (XEmacLite_Handler)EmacLiteSendHandler);

    // Enable the interrupts in the EmacLite controller.
    XEmacLite_EnableInterrupts(&ether);

    return XST_SUCCESS;
}

/******************************************************************************/
/*
 * The Ethernet receive interrupt handler. Called when a new message external 
 * arrives at the FPGA. The message is stored in an internal circular array and 
 * the pointer for new arrivals is incremented. Then the message is inspected 
 * to see whether it is a valid Ethernet type for the simulation. If it is not, 
 * the pointer is decremented back to its previous value. 
 * 
 * @param   callBackRef     The callback reference
 *
 * @return  None
 *
 ******************************************************************************/
static void EmacLiteRecvHandler(void *CallBackRef) {
    XEmacLite *XEmacInstancePtr;

    // Convert the argument to something useful.
    XEmacInstancePtr = (XEmacLite *)CallBackRef;

    // Handle the Receive callback.
    u8 extInputArrivalPtrOld = extInputArrivalPtr;
    extInputArrivalPtr = (extInputArrivalPtr + 1) % EXT_INPUT_SIZE;
    XEmacLite_Recv(XEmacInstancePtr, (u8 *)rawExternalInput[extInputArrivalPtrOld]);

    if ((rawExternalInput[extInputArrivalPtrOld][12] == 0x55) &&
            (rawExternalInput[extInputArrivalPtrOld][13] == 0xAA)) {
#ifdef DEBUG
        xil_printf("++ Receive Interrupt Triggered: Relevant (a%d, p%d) ++\n\r",
            extInputArrivalPtr, extInputProcessPtr);
#endif
    } else {
        extInputArrivalPtr--;
        XEmacLite_FlushReceive(&ether);     // Clear any received messages
    }
}

/******************************************************************************/
/*
 * The Ethernet transmit interrupt handler. Called when messages are sent 
 * externally to the FPGA. Although not used in this system, the method is 
 * needed if interrupt handlers are to be used for receiving Ethernet messages.  
 * 
 * @param   callBackRef     The callback reference
 *
 * @return  None
 *
 ******************************************************************************/
static void EmacLiteSendHandler(void *CallBackRef) {
#ifdef DEBUG
    print("++ Transmit Interrupt Triggered ++\n\r");
#endif

    XEmacLite *XEmacInstancePtr;

    // Convert the argument to something useful.
    XEmacInstancePtr = (XEmacLite *)CallBackRef;
}
