#include "xparameters.h"

#include <stdio.h>          // For simple input/output (I/O)
#include <stdlib.h>
#include <string.h>         // For memset()

#include "xuartlite_l.h"    // UART
#include "fsl.h"            // AXI Steam

#define ENTER   0x0A        // The ASCII code for '\n' or Line Feed (LF)

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

#define MODE_INIT               1   //
#define  CMD_PING               2   // Controller -> BoidCPU
#define CMD_PING_REPLY          3   // BoidCPU -> Controller
#define CMD_USER_INFO           4   // Controller -> BoidGPU
#define CMD_SIM_SETUP           5   // Controller -> Boid[CG]PU
#define MODE_CALC_NBRS          6   //
#define CMD_NBR_REPLY           8   // BoidCPU -> BoidCPU
#define MODE_POS_BOIDS          9   //
#define CMD_LOAD_BAL            10  // TODO: Decide on implementation
#define MODE_TRAN_BOIDS         11  //
#define CMD_BOID                12  // BoidCPU -> BoidCPU
#define MODE_DRAW               14  // TODO: Perhaps not needed?
#define CMD_DRAW_INFO           15  // BoidCPU -> BoidGPU
#define CMD_KILL                16  // Controller -> All

// BoidCPU definitions ---------------------------------------------------------
#define EDGE_COUNT              4   // The number of edges a BoidCPU has
#define MAX_BOIDCPU_NEIGHBOURS  8   // The maximum neighbours a BoidCPUs has

typedef enum { false, true } bool;

u32 data[MAX_CMD_BODY_LEN];
u32 to;
u32 from = CONTROLLER_ID;
u32 dataLength = 0;
u32 coords[EDGE_COUNT];

u32 inputData[MAX_CMD_LEN];

const char *commandDescriptions[16] = {
        "Initialisation mode",
        "Send ping to BoidCPUs",
        "Ping response from a BoidCPU",
        "User-inputed information for the BoidGPU",
        "Simulation setup information for a BoidCPU",
        "",
        "Calculate neighbours mode",
        "Reply of neighbouring BoidCPU's boids",
        "Position calculation mode",
        "Load balancing command",
        "Transfer boids mode",
        "Transmit a boid",
        "",
        "Draw mode",
        "Draw information heading to BoidGPU",
        "Kill simulation"};

void testInitMode();
void testPing();
void testPingReply();
void testUserInfo();
void testSimulationSetup();
void testNeighbourSearch();
void testNeighbourReply();
void testCalcNextBoidPos();
void testLoadBalance();
void testMoveBoids();
void testBoidCommand();
void testDrawBoids();
void testDrawInfo();
void testKillSwitch();

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data);
void chooseCommand(u8 commandID);
void printCommand(bool send, u32 *data);


int main() {
    // Setup Ethernet

    do {
        print("--------------------------------------------------\n\r");
        print("-------- FPGA Flocking Bird Test Harness ---------\n\r");
        print("--------------------------------------------------\n\r");

        bool cIDValid = false;  // True if the command ID entered is valid
        u8 cID = 0;            // The ID of the command to issue

        u8 index = 0;           // Index for keyPresses
        char keyPress;          // The key pressed
        char keyPresses[] = ""; // An array containing the keys pressed

        while(!cIDValid){       // Ask for a valid command ID
            index = 0;          // Reset the index and key press array
            memset(keyPresses, 0, sizeof(keyPresses));

            print("Choose a command from the following list: \n\r");
            int i = 0;
            for(i = 0; i < 15; i++) {
                xil_printf(" %d: %s\n\r", i + 1, commandDescriptions[i]);
            }

            do {
                keyPress = XUartLite_RecvByte(XPAR_RS232_UART_1_BASEADDR);
                XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, keyPress);

                keyPresses[index] = keyPress;
                index++;
                print("In while loop\n\r");

                if (keyPress == ENTER) {
                    xil_printf("0x%02x == 0x%02x", keyPress, ENTER);
                } else {
                    xil_printf("0x%02x != 0x%02x", keyPress, ENTER);
                }

            } while(keyPress != ENTER);     // Repeat while enter isn't pressed
            print("Exited while loop\n\r");
            cID = (u8)atoi(keyPresses);         // Convert the key pressed to int

            xil_printf("The number '%d' was received\n\r", cID);

            if ((cID >= 1) && (cID <= 16)) {
                cIDValid = true;
                print("Command is valid\n\r");

                // Send the command
                chooseCommand(cID);

                // Check for any response
//                int rv, invalid;
//                getfslx(rv, 0, FSL_NONBLOCKING);    // Non-blocking FSL read to device 0
//                fsl_isinvalid(invalid);             // Was there data ready?
//
//                if(!invalid) {
//                    print("Received data\n\r");
//                    inputData[0] = rv;
//                    int i = 0, value = 0;
//
//                    for (i = 1; i < inputData[0]; i++) {
//                        getfslx(value, 0, FSL_NONBLOCKING);
//                        inputData[i] = value;
//                    }
//                    printCommand(false, inputData);
//                } else {
//                    print("Not received data\n\r");
//                }

                int rv;
                print("Waiting for response...");
                getfslx(rv, 0, FSL_DEFAULT);
                print("received data: ");
                inputData[CMD_LEN] = rv;
                xil_printf("%d ", rv);

                int i = 0, value = 0;
                for (i = 0; i < inputData[CMD_LEN] - 1; i++) {
                    getfslx(value, 0, FSL_NONBLOCKING);
                    inputData[i + 1] = value;
                    xil_printf("%d ", value);
                }
                print("\n\r");
                printCommand(false, inputData);


            } else {
                print("\n\r**Error: Command ID must be between 1 and 16 inclusive. Please try again.\n\r");
            }
        }
    } while(1);

    return 0;
}


void chooseCommand(u8 commandID) {
    from = CONTROLLER_ID;

    switch(commandID) {
        case MODE_INIT:
            testInitMode();
            break;
        case CMD_PING:
            testPing();
            break;
        case CMD_PING_REPLY:
            testPingReply();
            break;
        case CMD_USER_INFO:
            testUserInfo();
            break;
        case CMD_SIM_SETUP:
            testSimulationSetup();
            break;
        case MODE_CALC_NBRS:
            testNeighbourSearch();
            break;
        case CMD_NBR_REPLY:
            testNeighbourReply();
            break;
        case MODE_POS_BOIDS:
            testCalcNextBoidPos();
            break;
        case CMD_LOAD_BAL:
            testLoadBalance();
            break;
        case MODE_TRAN_BOIDS:
            testMoveBoids();
            break;
        case CMD_BOID:
            testBoidCommand();
            break;
        case MODE_DRAW:
            testDrawBoids();
            break;
        case CMD_DRAW_INFO:
            testDrawInfo();
            break;
        case CMD_KILL:
            testKillSwitch();
            break;
        default:
            print("UNKNOWN COMMAND\n\r");
            break;
    }
}

void testInitMode() {
    // 4, 0, 1, 1 ||
    to = CMD_BROADCAST;
    dataLength = 0;
    createCommand(dataLength, to, from, MODE_INIT, data);
}

void testPing() {
    // Test ping response ----------------------------------------------------//
    // 4, 0, 1, 2 ||
    to = CMD_BROADCAST;
    dataLength = 0;
    createCommand(dataLength, to, from, CMD_PING, data);
}

void testPingReply() {
    // 6, 1, 42, 3 || 21, 123
    to = CONTROLLER_ID;
    from = 42;

    data[0] = 21;
    data[1] = 123;
    dataLength = 2;

    createCommand(dataLength, to, from, CMD_PING_REPLY, data);
}

void testUserInfo() {
    // 7, 2, 1, 4 || 21 42 84
    to = BOIDGPU_ID;
    data[0] = 21;
    data[1] = 42;
    data[2] = 84;
    dataLength = 3;

    createCommand(dataLength, to, from, CMD_USER_INFO, data);
}

// TODO: Need to send to broadcast during actual testing as random ID unknown
void testSimulationSetup() {
    // Test simulation setup ---------------------------------------------------
    // 18, 83, 1, 5 || 7, 10, 0, 0, 40, 40, 3, 4, 5, 8, 11, 10, 9, 6
    dataLength = 14;
    to = 83;            // The current random ID of the test BoidCPU

    u32 newID = 7;
    u32 initialBoidCount = 10;
    coords[0] = 0;
    coords[1] = 0;
    coords[2] = 40;
    coords[3] = 40;
    u32 neighbours[MAX_BOIDCPU_NEIGHBOURS] = {3, 4, 5, 8, 11, 10, 9, 6};

    data[0] = newID;
    data[1] = initialBoidCount;

    int i = 0;
    for (i = 0; i < EDGE_COUNT; i++) {
        data[2 + i] = coords[i];
    }

    for (i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        data[EDGE_COUNT + 2 + i] = neighbours[i];
    }

    createCommand(dataLength, to, from, CMD_SIM_SETUP, data);
}

void testNeighbourSearch() {
    // 4 0 1 6 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_CALC_NBRS, data);
}

void testNeighbourReply() {
    print("Testing neighbour reply - TODO\n\r");
}

void testCalcNextBoidPos() {
    // 4 0 1 9 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_POS_BOIDS, data);
}

void testLoadBalance() {
    // 4 0 1 10 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, CMD_LOAD_BAL, data);
}

void testMoveBoids() {
    // 4 0 1 11 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_TRAN_BOIDS, data);
}

void testBoidCommand() {
    print("Testing Boid command - TODO\n\r");
}

void testDrawBoids() {
    // 4 0 1 14 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_DRAW, data);
}

void testDrawInfo() {
    print("Testing draw info - TODO\n\r");
}

void testKillSwitch() {
    print("Testing kill switch - TODO\n\r");
}

void createCommand(u32 len, u32 to, u32 from, u32 type, u32 *data) {
    print("Creating command...");
    u32 command[MAX_CMD_LEN];

    command[CMD_LEN]  = len + CMD_HEADER_LEN;
    command[CMD_TO]   = to;
    command[CMD_FROM] = from;
    command[CMD_TYPE] = type;

    if (len > 0) {
        int i = 0;
        for(i = 0; i < len; i++) {
            command[CMD_HEADER_LEN + i] = data[i];
        }
    }
    print("done\n\r");

    printCommand(true, command);

    print("Sending command...");
    int i = 0;
    for (i = 0; i < CMD_HEADER_LEN + len; i++) {
        putfslx(command[i], 0, FSL_NONBLOCKING);
    }
    print("done\n\r");
}

void printCommand(bool send, u32 *data) {
    if(send) {
        if(data[CMD_TO] == CMD_BROADCAST) {
            print("-> TX, Controller sent broadcast: ");
        } else if(data[CMD_TO] == BOIDGPU_ID) {
            print("-> TX, Controller sent command to BoidGPU: ");
        } else {
            xil_printf("-> TX, Controller sent command to %d: ", data[CMD_TO]);
        }
    } else {
        if(data[CMD_TO] == CMD_BROADCAST) {
            // This should never happen - BoidCPUs should not be able to broadcast
            xil_printf("<- RX, Controller received broadcast from %d: ", data[CMD_FROM]);
        } else if(data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen
            print("<- RX, Controller received command from BoidGPU: ");
        } else {
            xil_printf("<- RX, Controller received command from %d: ", data[CMD_FROM]);
        }
    }

    switch(data[CMD_TYPE]) {
        case 0:
            print("do something");
            break;
        case MODE_INIT:
            print("initialise self");
            break;
        case CMD_PING:
            print("BoidCPU ping");
            break;
        case CMD_PING_REPLY:
            print("BoidCPU ping response");
            break;
        case CMD_USER_INFO:
            print("output user info");
            break;
        case CMD_SIM_SETUP:
            print("setup BoidCPU");
            break;
        case MODE_CALC_NBRS:
            print("calculate neighbours");
            break;
        case CMD_NBR_REPLY:
            print("neighbouring boids from neighbour");
            break;
        case MODE_POS_BOIDS:
            print("calculate new boid positions");
            break;
        case CMD_LOAD_BAL:
            print("load balance");
            break;
        case MODE_TRAN_BOIDS:
            print("transfer boids");
            break;
        case CMD_BOID:
            print("boid");
            break;
        case MODE_DRAW:
            print("send boids to BoidGPU");
            break;
        case CMD_DRAW_INFO:
            print("boid info heading to BoidGPU");
            break;
        case CMD_KILL:
            print("kill simulation");
            break;
        default:
            print("UNKNOWN COMMAND");
            break;
    }

    XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR, '\t');
    int i = 0;
    for(i = 0; i < CMD_HEADER_LEN; i++) {
    	xil_printf("%d ", data[i]);
    }
    print("|| ");

    for(i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
    	xil_printf("%d ", data[CMD_HEADER_LEN + i]);
    }
    print("\n\r");
}
