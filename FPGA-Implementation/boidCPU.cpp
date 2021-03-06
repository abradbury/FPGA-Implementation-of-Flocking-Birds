/** 
 * Copyright 2015 abradbury
 * 
 * boidCPU.cpp
 * 
 * This file represents a BoidCPU FPGA core. A BoidCPU is a processing unit 
 * responsible for an area of the simulation space. Any boids that are within 
 * a BoidCPU's bounds are managed and controlled by that BoidCPU. As boids move 
 * around the simaulation area they move between BoidCPU areas and so are 
 * transferred between BoidCPUs. 
 * 
 * By dividing the simulation space into multiple regions the complexity of the 
 * simulation is reduced as a boid would only need to query boids in 
 * neighbouring BoidCPUs to determine if they are neighbours, rather than 
 * querying all boids in the simulation space. This approach is based on the 
 * CoSMoS distributed continuous space design pattern proposed by Andrews et al. 
 * (http://www-users.cs.york.ac.uk/susan/bib/ss/nonstd/alife08a.pdf).
 * 
 * With the added complexity from the load balancing routines, the LUT (logic) 
 * usage of a BoidCPU core increased such that only 1 BoidCPU core could fit on 
 * an FPGA. To ensure 2 BoidCPUs on an FPGA, it is possible to reduce the boid 
 * model logic, resulting in sudden changes of direction, and disable load 
 * balancing using the REDUCED_LUT_USAGE and LOAD_BALANCING_ENABLED flags. 
 * 
 * This FPGA core was developed using the 2013.4 version Xilinx’s Vivado High 
 * Level Synthesis (HLS) Design Suite and deployed to multiple Xilinx Spartan-6 
 * LX45 FPGAs using Xilinx Platform Studio (XPS) 14.7.
 *
 ******************************************************************************/

/******************************* Include Files ********************************/

#include "boidCPU.h"

/**************************** Constant Definitions ****************************/

// #define USING_TESTBENCH          true    // Define when using HLS TestBench
// #define LOAD_BALANCING_ENABLED   true    // Define to enable load balancing
#define REDUCED_LUT_USAGE       true    // Define to reduce LUT usage

/**************************** Function Prototypes *****************************/

// Key function headers --------------------------------------------------------
static void simulationSetup(void);
static void calcNextBoidPositions(void);

#ifdef LOAD_BALANCING_ENABLED
static void evaluateLoad(void);
static void loadBalance(void);
#endif

static void calculateEscapedBoids(void);
static void updateDisplay(void);

void calculateBoidNeighbours(void);
void sendBoidsToNeighbours(void);
void processNeighbouringBoids(void);

// Supporting function headers -------------------------------------------------
void transmitBoids(uint16 *boidIDs, uint8 *recipientIDs, uint8 count);
void acceptBoid();

void packBoidsForSending(uint32 to, uint32 msg_type);
Boid parsePackedBoid(uint8 offset);

void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data);
bool fromNeighbour();

void commitAcceptedBoids();
void sendAck(uint8 type);

bool isBoidBeyond(Boid boid, uint8 edge);
bool isBoidBeyondSingle(Boid boid, uint8 edge);
bool isNeighbourTo(uint16 bearing);

// Debugging function headers --------------------------------------------------
void printCommand(bool send, uint32 *data);
void printStateOfBoidCPUBoids();

/**************************** Variable Definitions ****************************/

// BoidCPU variables -----------------------------------------------------------
int8 boidCPUID = FIRST_BOIDCPU_ID;
int12 boidCPUCoords[4];

uint11 simulationWidth  = 0;
uint11 simulationHeight = 0;

uint8 neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS];
// True when neighbouring BoidCPUs have completed their setup
bool neighbouringBoidCPUsSetup = false;

// The number of distinct neighbouring BoidCPUs
uint8 distinctNeighbourCount = 0;
uint8 distinctNeighbourCounter = 0;      // A counter to the above

int16 queuedBoids[MAX_QUEUED_BOIDS][5];  // Holds boids received from neighbours
uint8 queuedBoidsCounter = 0;            // A counter for queued boids

uint32 inputData[MAX_CMD_LEN];
uint32 outputData[MAX_OUTPUT_CMDS][MAX_CMD_LEN];
uint32 outputBody[30];
uint8 outputCount = 0;                   // The number of output messages stored

// Boid variables --------------------------------------------------------------
uint8 boidCount;
Boid boids[MAX_BOIDS];     // TODO: Perhaps re-implement as a LL due to deletion
Boid *boidNeighbourList[MAX_BOIDS][MAX_NEIGHBOURING_BOIDS];

// A list of possible neighbouring boids for the BoidCPU
Boid possibleBoidNeighbours[MAX_NEIGHBOURING_BOIDS];
uint8 possibleNeighbourCount = 0;        // Number of possible boid neighbours

// Debugging variables ---------------------------------------------------------
bool continueOperation = true;

/******************************************************************************/
/*
 * The top level function of the BoidCPU core - containing the only external
 * interfaces of the whole core. Continually checks for input data and when it
 * arrives it is stored and a function is called to process the data. After the
 * called function has been called the output buffer is checked. If there is
 * output ready to send it is sent externally, else the function waits for
 * more input. The input and output ports are implemented as a FIFO structure
 * and can hold one value - so data needs to be read/sent promptly.
 *
 * @param   input   The AXI-bus import port of the BoidCPU core
 * @param   output  The AXI-bus output port of the BoidCPU core
 *
 * @return  None
 *
 ******************************************************************************/
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port = input
#pragma HLS INTERFACE ap_fifo port = output
#pragma HLS RESOURCE variable = input core = AXI4Stream
#pragma HLS RESOURCE variable = output core = AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port = return

    // Continually check for input and deal with it. Note that reading an empty
    // input stream will generate warnings in HLS, but should be blocking in the
    // actual implementation.

#ifdef USING_TESTBENCH
    inputData[CMD_LEN] = input.read();
#endif

    mainWhileLoop: while (continueOperation) {
        // INPUT ---------------------------------------------------------------
        // Block until there is input available
#ifndef USING_TESTBENCH
        inputData[CMD_LEN] = input.read();
#endif

        // When there is input, read in the command
        inputLoop: for (int i = 1; i < inputData[CMD_LEN]; i++) {
            inputData[i] = input.read();
        }
        printCommand(false, inputData);
        // ---------------------------------------------------------------------

        // STATE CHANGE --------------------------------------------------------
        if ((inputData[CMD_FROM] != boidCPUID) &&
                ((inputData[CMD_TO] == boidCPUID) ||
                        (inputData[CMD_TO] == CMD_BROADCAST) ||
                        fromNeighbour())) {
            switch (inputData[CMD_TYPE]) {
            case CMD_SIM_SETUP:
                simulationSetup();
                break;
            case MODE_CALC_NBRS:
                sendBoidsToNeighbours();
                break;
            case CMD_NBR_REPLY:
                processNeighbouringBoids();
                break;
            case MODE_POS_BOIDS:
                calcNextBoidPositions();
                break;
#ifdef LOAD_BALANCING_ENABLED
            case MODE_LOAD_BAL:
                evaluateLoad();
                break;
            case CMD_LOAD_BAL:
                loadBalance();
                break;
#endif
            case MODE_TRAN_BOIDS:
                calculateEscapedBoids();
                break;
            case CMD_BOID:
                acceptBoid();
                break;
            case MODE_DRAW:
                updateDisplay();
                break;
            default:
                std::cout << "Command state " << inputData[CMD_TYPE] <<
                " not recognised" << std::endl;
                break;
            }
        } else {
            std::cout << "The above message was ignored" << std::endl;
        }
        // ---------------------------------------------------------------------

        // OUTPUT --------------------------------------------------------------
        // If there is output to send, send it
        if (outputCount > 0) {
            outerOutLoop: for (int j = 0; j < outputCount; j++) {
                innerOutLoop: for (int i = 0; i < outputData[j][CMD_LEN]; i++) {
                    output.write(outputData[j][i]);
                }
                printCommand(true, outputData[j]);
            }
        }
        outputCount = 0;
        // ---------------------------------------------------------------------

#ifdef USING_TESTBENCH
        continueOperation = input.read_nb(inputData[0]);
#endif
    }
    std::cout << "=============BoidCPU has finished==============" << std::endl;
}

//==============================================================================
// State functions =============================================================
//==============================================================================

/******************************************************************************/
/*
 * Sets ID to that provided by the controller. Populates list of neighbouring
 * BoidCPUs. Initialises pixel coordinates and edges. Creates own boids.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void simulationSetup() {
    std::cout << "-Preparing BoidCPU for simulation..." << std::endl;

    // Set BoidCPU parameters (supplied by the controller)
    int8 oldBoidCPUID = boidCPUID;
    boidCPUID = inputData[CMD_HEADER_LEN + CMD_SETUP_NEWID_IDX];
    boidCount = inputData[CMD_HEADER_LEN + CMD_SETUP_BDCNT_IDX];

    edgeSetupLoop: for (int i = 0; i < EDGE_COUNT; i++) {
        boidCPUCoords[i] = inputData[CMD_HEADER_LEN + CMD_SETUP_COORD_IDX + i];
    }

    // Get the number of distinct neighbours
    distinctNeighbourCount = inputData[CMD_HEADER_LEN + CMD_SETUP_NBCNT_IDX];

    // Get the list of neighbours on each edge
    neighbourSetupLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        neighbouringBoidCPUs[i] =
                inputData[CMD_HEADER_LEN + CMD_SETUP_BNBRS_IDX + i];
    }
    neighbouringBoidCPUsSetup = true;

    // Get the simulation width and height
    simulationWidth  = inputData[CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX];
    simulationHeight = inputData[CMD_HEADER_LEN + CMD_SETUP_SIMWH_IDX + 1];

    // Print out BoidCPU parameters
    std::cout << "BoidCPU #" << oldBoidCPUID << " now has ID #" <<
            boidCPUID << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " initial boid count: " <<
            boidCount << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " has " << distinctNeighbourCount
            << " distinct neighbouring BoidCPUs" << std::endl;

    std::cout << "BoidCPU #" << boidCPUID << " coordinates: [";
    printEdgeLoop: for (int i = 0; i < EDGE_COUNT; i++) {
        std::cout << boidCPUCoords[i] << ", ";
    } std::cout << "]" << std::endl;
    std::cout << "BoidCPU #" << boidCPUID << " neighbours: [";
    printNeighbourLoop: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        std::cout << neighbouringBoidCPUs[i] << ", ";
    } std::cout << "]" << std::endl;

    std::cout << "The simulation is of width " << simulationWidth <<
            " and of height " << simulationHeight << std::endl;

    // Create the boids
    uint16 boidID;
    int12 widthStep  = (boidCPUCoords[2] - boidCPUCoords[0]) / boidCount;
    int12 heightStep = (boidCPUCoords[3] - boidCPUCoords[1]) / boidCount;

#ifdef REDUCED_LUT_USAGE
    // FIXME: Only works for BoidCPUs less than MAX_VELOCITY * 2
    int4 initialSpeed = -MAX_VELOCITY + boidCPUID;
#else
    // This could be used to add some variance to the velocities of the boids
    int16_fp velStep = int16_fp(MAX_VELOCITY + MAX_VELOCITY) / boidCount;
#endif

    boidCreationLoop: for (int i = 0; i < boidCount; i++) {
#ifdef REDUCED_LUT_USAGE
        Vector velocity = Vector(initialSpeed, -initialSpeed);

        Vector position = Vector((widthStep * i) + boidCPUCoords[0] + 1,
                (heightStep * i) + boidCPUCoords[1] + 1);
#else
        Vector velocity = Vector(-MAX_VELOCITY + (velStep * i) + boidCPUID,
                MAX_VELOCITY - (velStep * i));

        // This would introduce some variance into the positions of the boids
        int16_fp xPos = (widthStep * i) + boidCPUCoords[0] + 1;
        if (int4(xPos) < 0) xPos = xPos + boidCPUID + boidCPUID + boidCPUID;

        Vector position = Vector(xPos, (heightStep * i) + boidCPUCoords[1] + 1);
#endif

        boidID = ((boidCPUID - 1) * boidCount) + i + 1;

        Boid boid = Boid(boidID, position, velocity);
        boids[i] = boid;
    }

    // Send ACK signal
    sendAck(CMD_SIM_SETUP);
}

/******************************************************************************/
/*
 * Send this BoidCPU's boids to its neighbouring BoidCPUs so they can use them
 * in calculating neighbours for their boids. Splits the data to be sent into
 * multiple messages if it exceeds the maximum command data size.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void sendBoidsToNeighbours() {
    std::cout << "-Sending boids to neighbouring BoidCPUs..." << std::endl;

    packBoidsForSending(CMD_MULTICAST, CMD_NBR_REPLY);

#ifndef REDUCED_LUT_USAGE
    // If there is just one BoidCPU - should not happen (often) if LUT usage
    // is reduced as at least 2 BoidCPUs can fit on a single Atlys board
    // This is quite a costly operation.
    if (distinctNeighbourCount == 0) {
        addOwnBoidsToNbrListZero: for (int i = 0; i < boidCount; i++) {
            possibleBoidNeighbours[possibleNeighbourCount] = boids[i];
            possibleNeighbourCount++;
        }

        calculateBoidNeighbours();

        // Send ACK signal
        sendAck(MODE_CALC_NBRS);
    }
#endif
}

/******************************************************************************/
/*
 * When boids are received from neighbouring BoidCPUs, process them and add
 * them to a list of possible neighbouring boids. These are used to calculate
 * the neighbours of boids contained within this BoidCPU.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void processNeighbouringBoids() {
    // Before processing first response, add own boids to list
    if (distinctNeighbourCounter == 0) {
        addOwnBoidsToNbrList: for (int i = 0; i < boidCount; i++) {
            possibleBoidNeighbours[possibleNeighbourCount] = boids[i];
            possibleNeighbourCount++;
        }
    }

    // Calculate the number of boids per message TODO: Remove division
    uint8 boidsPerMsg = (inputData[CMD_LEN] - CMD_HEADER_LEN - 1) /
            BOID_DATA_LENGTH;

    std::cout << "-BoidCPU #" << boidCPUID << " received " << boidsPerMsg <<
            " boids from BoidCPU #" << inputData[CMD_FROM] << std::endl;

    // Parse each received boid and add to possible neighbour list
    rxNbrBoidLoop: for (int i = 0; i < boidsPerMsg; i++) {
        possibleBoidNeighbours[possibleNeighbourCount] = parsePackedBoid(i);

        // Don't go beyond the edge of the array
        if (possibleNeighbourCount != MAX_NEIGHBOURING_BOIDS) {
            possibleNeighbourCount++;
        } else {
            break;
        }
    }

    // If no further messages are expected, then process it
    if (inputData[CMD_HEADER_LEN + 0] == 0) {
        distinctNeighbourCounter++;

        if (distinctNeighbourCounter == distinctNeighbourCount) {
            calculateBoidNeighbours();

            // Send ACK signal
            sendAck(MODE_CALC_NBRS);
        }
    } else {
        std::cout << "Expecting " << inputData[CMD_HEADER_LEN + 0] << \
                " further message(s) from " << inputData[CMD_FROM] << std::endl;
    }
}

/******************************************************************************/
/*
 * This is called when the BoidCPUs are instructed to calculate the next
 * positions of the boids. It was in the neighbour search stage, but it became
 * complex to determine when all the boids from neighbours had been received
 * due to splitting of data over multiple messages and replicated neighbours.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void calculateBoidNeighbours() {
    outerCalcBoidNbrsLoop: for (int i = 0; i < boidCount; i++) {
        uint8 boidNeighbourCount = 0;
        inCalcBoidNbrsLoop: for (int j = 0; j < possibleNeighbourCount; j++) {
            if (possibleBoidNeighbours[j].id != boids[i].id) {
                int32_fp boidSeparation = Vector::squaredDistanceBetween(
                        boids[i].position, possibleBoidNeighbours[j].position);

                if (boidSeparation < VISION_RADIUS_SQUARED) {
                    boidNeighbourList[i][boidNeighbourCount] =
                            &possibleBoidNeighbours[j];
                    boidNeighbourCount++;
                }
            }
        }

        boids[i].setNeighbourDetails(i, boidNeighbourCount);
    }

    // Reset the flags
    possibleNeighbourCount = 0;
    distinctNeighbourCounter = 0;
}

/******************************************************************************/
/*
 * Iterates through the boids contained in this BoidCPU and updates their
 * position based on the model of flocking birds used. If, after a boid is
 * updated, its new position is outside the simulation area, the boid's
 * position is wrapped around. When all the boids have been updated an ACK is
 * issued.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void calcNextBoidPositions() {
    std::cout << "-Calculating next boid positions..." << std::endl;

    updateBoidsLoop: for (int i = 0; i < boidCount; i++) {
        boids[i].update();

        // Contain boid pixel position values to within the simulation area
        if (boids[i].position.x > simulationWidth) {
            boids[i].position.x = 0;
        } else if (boids[i].position.x < 0) {
            boids[i].position.x = simulationWidth;
        }

        if (boids[i].position.y > simulationHeight) {
            boids[i].position.y = 0;
        } else if (boids[i].position.y < 0) {
            boids[i].position.y = simulationHeight;
        }
    }

    // Send ACK signal
    sendAck(MODE_POS_BOIDS);
}

#ifdef LOAD_BALANCING_ENABLED
/******************************************************************************/
/*
 * If the number of boids contained within this BoidCPU is greater than the 
 * boid threshold, signal the controller. Else, just send an acknowledgement.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void evaluateLoad() {
    if (boidCount > BOID_THRESHOLD) {
        std::cout << "-Load balancing..." << std::endl;

        generateOutput(0, CONTROLLER_ID, CMD_LOAD_BAL_REQUEST, outputBody);
    } else {
        std::cout << "-No need to load balance" << std::endl;
        sendAck(MODE_LOAD_BAL);
    }
}

/******************************************************************************/
/*
 * Called on receiving a load balance command from the BoidMaster. Parses the 
 * received instructions and changes the BoidCPU's boundaries as needed. If the 
 * change causes one of the BoidCPU's boundaries to become minimal, inform the 
 * BoidMaster. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void loadBalance() {
    int16 edgeChanges = (int16)inputData[CMD_HEADER_LEN + 0];

    std::cout << "BoidCPU #" << boidCPUID << " changing NORTH edge from " <<
            boidCPUCoords[Y_MIN];
    boidCPUCoords[Y_MIN] += VISION_RADIUS * int4(edgeChanges >> NORTH_IDX);
    std::cout << " to " << boidCPUCoords[Y_MIN] << std::endl;

    std::cout << "BoidCPU #" << boidCPUID << " changing EAST edge from " <<
            boidCPUCoords[X_MAX];
    boidCPUCoords[X_MAX] += VISION_RADIUS * int4(edgeChanges >> EAST_IDX);
    std::cout << " to " << boidCPUCoords[X_MAX] << std::endl;

    std::cout << "BoidCPU #" << boidCPUID << " changing SOUTH edge from " <<
            boidCPUCoords[Y_MAX];
    boidCPUCoords[Y_MAX] += VISION_RADIUS * int4(edgeChanges >> SOUTH_IDX);
    std::cout << " to " << boidCPUCoords[Y_MAX] << std::endl;

    std::cout << "BoidCPU #" << boidCPUID << " changing WEST edge from " <<
            boidCPUCoords[X_MIN];
    boidCPUCoords[X_MIN] += VISION_RADIUS * int4(edgeChanges >> WEST_IDX);
    std::cout << " to " << boidCPUCoords[X_MIN] << std::endl;

    // Is minimal?
    int12 width  = boidCPUCoords[2] - boidCPUCoords[0];
    int12 height = boidCPUCoords[3] - boidCPUCoords[1];
    if ((width <= VISION_RADIUS) && (height <= VISION_RADIUS)) {
        std::cout << "BoidCPU #" << boidCPUID << " minimal" << std::endl;
        outputBody[0] = 2;
        generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
    } else if (width <= VISION_RADIUS) {
        std::cout << "BoidCPU #" << boidCPUID << " width minimal" << std::endl;
        outputBody[0] = 0;
        generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
    } else if (height <= VISION_RADIUS) {
        std::cout << "BoidCPU #" << boidCPUID << " height minimal" << std::endl;
        outputBody[0] = 1;
        generateOutput(1, CONTROLLER_ID, CMD_BOUNDS_AT_MIN, outputBody);
    }
}
#endif

/******************************************************************************/
/*
 * Sends information about the boids contained within this BoidCPU to the
 * BoidGPU for drawing. Before this is done any boids that arrived during the
 * transfer stage of the simulation are committed.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void updateDisplay() {
    if (queuedBoidsCounter > 0) {
        commitAcceptedBoids();

        std::cout << "-Updating display" << std::endl;
        packBoidsForSending(BOIDGPU_ID, CMD_DRAW_INFO);
    } else {
        std::cout << "-Updating display" << std::endl;
        packBoidsForSending(BOIDGPU_ID, CMD_DRAW_INFO);
    }
}

//==============================================================================
//- Boid Transmission and Acceptance -------------------------------------------
//==============================================================================

/******************************************************************************/
/*
 * Called after new boid positions have been calculated load balancing has 
 * occurred. Any boids that are now outside of the current BoidCPU's bounds are 
 * transferred to a neighbouring BoidCPU. 
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void calculateEscapedBoids() {
    std::cout << "-Transferring boids..." << std::endl;

    uint16 boidIDs[MAX_BOIDS];
    uint8 recipientIDs[MAX_BOIDS];
    uint8 counter = 0;

    // For each boid
    moveBoidsLoop: for (int i = 0; i < boidCount; i++) {
        // For each bearing (from NORTHWEST (0) to WEST (7))
        bearLoop: for (int bearing = NORTHWEST; bearing < WEST + 1; bearing++) {
            // If a BoidCPU is at the bearing & boid is beyond the bearing limit
            if (isNeighbourTo(bearing) && isBoidBeyond(boids[i], bearing)) {
                // Mark boid as to be transferred
                boidIDs[counter] = boids[i].id;
                recipientIDs[counter] = neighbouringBoidCPUs[bearing];
                counter++;
            }
        }
    }

    if (counter > 0) {
        transmitBoids(boidIDs, recipientIDs, counter);
    } else {
        sendAck(MODE_TRAN_BOIDS);
    }
}

/******************************************************************************/
/*
 * Checks if the supplied boid is beyond the supplied BoidCPU edge. Can handle
 * compound edge bearings such as NORTHWEST.
 * 
 * TODO: Every case statement is the same (NORTH and WEST) - investigate when 
 *  next have access to FPGA kit - can't recall issues with this.
 *
 * @param   boid    The boid to check bounds for
 * @param   edge    The edge to check that the boid is beyond
 *
 * @return          True if the boid is beyond the edge, false otherwise
 *
 ******************************************************************************/
bool isBoidBeyond(Boid boid, uint8 edge) {
    bool result;

    switch (edge) {
    case NORTHWEST:
        result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
        break;
    case NORTHEAST:
        result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
        break;
    case SOUTHEAST:
        result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
        break;
    case SOUTHWEST:
        result = isBoidBeyondSingle(boid, NORTH) && isBoidBeyondSingle(boid, WEST);
        break;
    default:
        result = isBoidBeyondSingle(boid, edge);
        break;
    }

    return result;
}

/******************************************************************************/
/*
 * Checks if the supplied boid is beyond the supplied BoidCPU edge. Can only
 * handle singular edge bearings e.g. NORTH.
 *
 * @param   boid    The boid to check bounds for
 * @param   edge    The edge to check that the boid is beyond
 *
 * @return          True if the boid is beyond the edge, false otherwise
 *
 ******************************************************************************/
bool isBoidBeyondSingle(Boid boid, uint8 edge) {
    int16_fp coordinate;
    bool result;

    switch (edge) {
    case NORTH:
        edge = Y_MIN;
        break;
    case EAST:
        edge = X_MAX;
        break;
    case SOUTH:
        edge = Y_MAX;
        break;
    case WEST:
        edge = X_MIN;
        break;
    default:
        break;
    }

    // Determine the coordinate to check against
    if ((edge == X_MIN) || (edge == X_MAX)) {
        coordinate = boid.position.x;
    } else if ((edge == Y_MIN) || (edge == Y_MAX)) {
        coordinate = boid.position.y;
    }

    // Determine if the comparison should be less than or greater than
    if ((edge == X_MIN) || (edge == Y_MIN)) {
        if (coordinate < boidCPUCoords[edge]) {
            result = true;
        } else {
            result = false;
        }
    } else if ((edge == X_MAX) || (edge == Y_MAX)) {
        if (coordinate > boidCPUCoords[edge]) {
            result = true;
        } else {
            result = false;
        }
    }

    return result;
}

/******************************************************************************/
/*
 * Checks if the BoidCPU has a neighbour at the specified bearing
 *
 * @param   bearing     From NORTHWEST (0) around a compass to WEST (7)
 *
 * @return              True if the BoidCPU has a neighbour at the specified
 *                       bearing, false otherwise
 *
 ******************************************************************************/
bool isNeighbourTo(uint16 bearing) {
    if (neighbouringBoidCPUs[bearing] > 0) {
        return true;
    } else {
        return false;
    }
}

/******************************************************************************/
/*
 * Called after boids in a BoidCPU have been identified for transportation to 
 * neighbouring BoidCPUs. 
 *
 * @param   boidIDs         An array of the IDs of boids to transfer
 * @param   recipientIDs    An array of the IDs of the neighbouring BoidCPUs
 * @param   count           The number of boids to transfer
 *
 * @return  None
 *
 ******************************************************************************/
void transmitBoids(uint16 *boidIDs, uint8 *recipientIDs, uint8 count) {
    // First transmit all the boids
    boidTransmitLoop: for (int i = 0; i < count; i++) {
        boidTransmitSearchLoop: for (int j = 0; j < boidCount; j++) {
            if (boidIDs[i] == boids[j].id) {
                // TODO: Perhaps move this to the boid class?
                outputBody[0] = boids[j].id;
                outputBody[1] = boids[j].position.x;
                outputBody[2] = boids[j].position.y;
                outputBody[3] = boids[j].velocity.x;
                outputBody[4] = boids[j].velocity.y;

                generateOutput(5, recipientIDs[i], CMD_BOID, outputBody);

                std::cout << "-Transferring boid #" << boids[j].id <<
                        " to boidCPU #" << recipientIDs[i] << std::endl;

                break;
            }
        }
    }

    // Then delete the boids from the BoidCPUs own boid list
    outerBoidRemovalLoop: for (int i = 0; i < count; i++) {
        bool boidFound = false;
        //  'j < boidCount - 1' used as list is decremented by 1
        innerBoidRemovalLoop: for (int j = 0; j < boidCount - 1; j++) {
            if (boids[j].id == boidIDs[i]) {
                boidFound = true;
            }

            if (boidFound) {
                boids[j] = boids[j + 1];
            }
        }
        boidCount--;
    }

    // Send ACK signal
    sendAck(MODE_TRAN_BOIDS);
}

/******************************************************************************/
/*
 * Called when a BoidCPU receives a boid from a neighbouring BoidCPU. 
 * 
 * Incoming boids are stored in a temporary structure as in this phase of the 
 * simulation BoidCPUs would be transferring and deleting boids from their lists 
 * and inserting a new boid whilst this is happening leads to issues. 
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void acceptBoid() {
    // TODO: Replace 5 with BOID_DATA_LENGTH when using common transmission
    if (queuedBoidsCounter < (MAX_QUEUED_BOIDS - 1)) {
        queueBoidsLoop: for (int i = 0; i < 5; i++) {
            queuedBoids[queuedBoidsCounter][i] = inputData[CMD_HEADER_LEN + i];
        }

        queuedBoidsCounter++;
    }
}

/******************************************************************************/
/*
 * Commits boids that have been accepted by the current BoidCPU. 
 * 
 * Called on the UPDATE_DISPLAY stage of the simulation to ensure that all 
 * BoidCPUs have finished sending their boids to neighbours. 
 *
 * @param   None
 * 
 * @return  None
 *
 ******************************************************************************/
void commitAcceptedBoids() {
    std::cout << "-Committing accepted boids..." << std::endl;

    commitQueuedBoidLoop: for (int i = 0; i < queuedBoidsCounter; i++) {
        if (boidCount < (MAX_BOIDS - 1)) {
            // TODO: Replace 5 with BOID_DATA_LENGTH when using common transmission
            uint16 boidID = queuedBoids[i][0];
            Vector boidPosition = Vector(queuedBoids[i][1], queuedBoids[i][2]);
            Vector boidVelocity = Vector(queuedBoids[i][3], queuedBoids[i][4]);
            Boid boid = Boid(boidID, boidPosition, boidVelocity);

            boids[boidCount] = boid;
            boidCount++;

            std::cout << "-BoidCPU #" << boidCPUID << " accepted boid #" << boidID
                    << " from boidCPU #" << inputData[CMD_FROM] << std::endl;
        }
    }

    queuedBoidsCounter = 0;
}

//============================================================================//
//- Supporting functions -----------------------------------------------------//
//============================================================================//

/******************************************************************************/
/*
 * A debug function used to print the state of a BoidCPUs boids.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void printStateOfBoidCPUBoids() {
    boidStatePrintLoop: for (int i = 0; i < boidCount; i++) {
        std::cout << "Boid " << boids[i].id << " has position [" <<
                boids[i].position.x << ", " << boids[i].position.y <<
                "] and velocity [" << boids[i].velocity.x << ", " <<
                boids[i].velocity.y << "]" << std::endl;
    }
}

/******************************************************************************/
/*
 * Sends an acknowledgement (ACK) message with the current state of the 
 * simulation. Used by the BoidMaster to synchronise the state of the 
 * simulation.
 *
 * @param   type    The state of the simulation to acknowledge
 *
 * @return  None
 *
 ******************************************************************************/
void sendAck(uint8 type) {
    outputBody[0] = type;
    generateOutput(1, CONTROLLER_ID, CMD_ACK, outputBody);
}

/******************************************************************************/
/*
 * Parses a recived boid that was packed for transmission. Returns a Boid 
 * instance derived from the packed boid data. 
 *
 * @param   offset  The start of the boid data in the input array
 *
 * @return          A Boid instance of the parsed boid data
 *
 ******************************************************************************/
Boid parsePackedBoid(uint8 offset) {
    uint8 index = CMD_HEADER_LEN + (BOID_DATA_LENGTH * offset);

    uint32 pos = inputData[index + 1];
    uint32 vel = inputData[index + 2];
    uint16 bID = inputData[index + 3];      // boid ID

    // Decode position and velocity
    Vector position = Vector(((int32_fp)((int32)pos >> 16)) >> 4,
            ((int32_fp)((int16)pos)) >> 4);

    Vector velocity = Vector(((int32_fp)((int32)vel >> 16)) >> 4,
            ((int32_fp)((int16)vel)) >> 4);

    std::cout << "-BoidCPU #" << boidCPUID << " received boid #" << bID <<
            " from BoidCPU #" << inputData[CMD_FROM] << std::endl;

    return Boid(bID, position, velocity);
}

/******************************************************************************/
/*
 * Uses bitshifting to reduce the amount of data that is communicated. This is 
 * done by packing the boid data, which is currently 16 bits each, into the 32 
 * bit fields used when communicating over the AXI-bus. Splits the boids of a 
 * BoidCPU across multiple messages if they do not fit in one and can encode 
 * negative and fixed-point values. If the BoidCPU contains no boids, an empty 
 * message is sent so the recipient knows this. 
 * 
 * TODO: Currently sends all boids of a BoidCPU (for neighbour search and 
 * BoidGPU update), enhance to specify what boids to send (for boid transfer)
 *
 * @param   to          The recipient of the message
 * @param   msg_type    The type of message to send
 *
 * @return  None
 *
 ******************************************************************************/
void packBoidsForSending(uint32 to, uint32 msg_type) {
    if (boidCount > 0) {
        // The first bit of the body is used to indicate the number of messages
        uint16 partialMaxCmdBodyLen = MAX_CMD_BODY_LEN - 1;

        // First, calculate how many messages need to be sent
        // Doing this division saves a DSP at the expense of about 100 LUTs
        int16 numerator = boidCount * BOID_DATA_LENGTH;
        uint16 msgCount = 0;
        nbrMsgCountCalcLoop: for (msgCount = 0; numerator > 0; msgCount++) {
            numerator -= partialMaxCmdBodyLen;
        }

        // Then calculate the number of boids that can be sent per message
        uint16 boidsPerMsg = (uint16)(partialMaxCmdBodyLen / BOID_DATA_LENGTH);

        // Determine the initial boid indexes for this message
        uint8 startBoidIndex = 0;
        uint8 endBoidIndex   = startBoidIndex + boidsPerMsg;

        // Next, send a message for each group of boids
        nbrMsgSendLoop: for (uint16 i = 0; i < msgCount; i++) {
            // Limit the end index if the message won't be full
            if (endBoidIndex > boidCount) {
                endBoidIndex = boidCount;
            }

            // Put the number of subsequent messages in the first body field
            outputBody[0] = msgCount - i - 1;

            // The next step is to create the message data
            uint8 index = 1;
            NMClp: for (uint8 j = startBoidIndex; j < endBoidIndex; j++) {
                uint32 position = 0;
                uint32 velocity = 0;

                // Encode the boid position and velocity -----------------------
                // First, cast the int16_fp value to an int32_fp value. This
                // enables up to 24 bits of integer values (and 8 fractional
                // bits). Then, shift the value left by 4 bits to bring the
                // fractional bits into the integer bit range. This is needed
                // because casting an int16_fp straight to an integer causes
                // the fractional bits to be lost. After casting to a uint32
                // (the transmission data type) shift the x value to the top
                // 16 bits of the variable. The y value is placed in the bottom
                // 16 bits of the variable, but if this is negative a mask
                // needs to be applied to clear the 1s in the top 16 bits that
                // are there due to the 2s complement notation for negatives.

                // Encode position
                position |= ((uint32)(((int32_fp)(boids[j].position.x)) << 4) << 16);

                if (boids[j].position.y < 0) {
                    position |= ((~(((uint32)0xFFFF) << 16)) &
                            ((uint32)((int32_fp)(boids[j].position.y) << 4)));
                } else {
                    position |= ((uint32)((int32_fp)(boids[j].position.y) << 4));
                }

                // Encode velocity
                velocity |= ((uint32)(((int32_fp)(boids[j].velocity.x)) << 4) << 16);

                if (boids[j].velocity.y < 0) {
                    velocity |= ((~(((uint32)0xFFFF) << 16)) &
                            ((uint32)((int32_fp)(boids[j].velocity.y) << 4)));
                } else {
                    velocity |= ((uint32)((int32_fp)(boids[j].velocity.y) << 4));
                }

                outputBody[index + 0] = position;
                outputBody[index + 1] = velocity;
                // ID can be removed on deployment
                outputBody[index + 2] = boids[j].id;

                index += BOID_DATA_LENGTH;
            }

            // Finally send the message
            uint32 dataLength = (((endBoidIndex - startBoidIndex)) * BOID_DATA_LENGTH) + 1;
            generateOutput(dataLength, to, msg_type, outputBody);

            // Update the boid indexes for the next message
            startBoidIndex += boidsPerMsg;
            endBoidIndex = startBoidIndex + boidsPerMsg;
        }
    } else {
        std::cout << "No boids to send, sending empty message" << std::endl;
        outputBody[0] = 0;
        generateOutput(1, to, msg_type, outputBody);
    }
}

/******************************************************************************/
/*
 * Takes data to be transmitted and places it in an queue of data. This queue 
 * is processed, i.e. the elements sent, when the control of the program 
 * returns to the top-level function. This is because no other function has 
 * access to the input and output ports. If the output queue is full, the 
 * new data is not added.
 *
 * @param   len     The length of the message body
 * @param   to      The recipient of the message
 * @param   type    The type of the message (defined in boidCPU.h)
 * @param   data    The message data
 *
 * @return  None
 *
 ******************************************************************************/
void generateOutput(uint32 len, uint32 to, uint32 type, uint32 *data) {
    if (outputCount > MAX_OUTPUT_CMDS - 1) {
        std::cout << "Cannot send message, output buffer is full (" <<
                outputCount << "/" << MAX_OUTPUT_CMDS << ")" << std::endl;
    } else {
        outputData[outputCount][CMD_LEN]  = len + CMD_HEADER_LEN;
        outputData[outputCount][CMD_TO]   = to;
        outputData[outputCount][CMD_FROM] = boidCPUID;
        outputData[outputCount][CMD_TYPE] = type;

        if (len > 0) {
            createOutputCommandLoop: for (int i = 0; i < len; i++) {
                outputData[outputCount][CMD_HEADER_LEN + i] = data[i];
            }
        }
        outputCount++;
    }
}

/******************************************************************************/
/*
 * Iterate through the list of neighbouring BoidCPUs to determine whether the
 * message received was from one of the neighbours. Return true if it was and
 * return false otherwise.
 *
 * TODO: Would it be better to return as soon as true is set?
 *
 * @param   None
 *
 * @return          True if the message was from a neighbour, false otherwise
 *
 ******************************************************************************/
bool fromNeighbour() {
    bool result = false;

    if (neighbouringBoidCPUsSetup) {
        fromNbrCheck: for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
            if (inputData[CMD_FROM] == neighbouringBoidCPUs[i]) {
                result = true;
            }
        }
    }

    return result;
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
 void printCommand(bool send, uint32 *data) {
    if (send) {
        if (data[CMD_TO] == CONTROLLER_ID) {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to controller: ";
        } else if (data[CMD_TO] == BOIDGPU_ID) {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to BoidGPU: ";
        } else {
            std::cout << "-> TX, BoidCPU #" << boidCPUID << " sent command to " << data[CMD_TO] << ": ";
        }
    } else {
        if (data[CMD_FROM] == CONTROLLER_ID) {
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from controller: ";
        } else if (data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from BoidGPU: ";
        }  else {
            std::cout << "<- RX, BoidCPU #" << boidCPUID << " received command from " << data[CMD_FROM] << ": ";
        }
    }

    switch (data[CMD_TYPE]) {
    case 0:
        std::cout << "do something";
        break;
    case MODE_INIT:
        std::cout << "initialise self";
        break;
    case CMD_PING:
        std::cout << "BoidCPU ping";
        break;
    case CMD_PING_REPLY:
        std::cout << "BoidCPU ping response";
        break;
    case CMD_USER_INFO:
        std::cout << "output user info";
        break;
    case CMD_SIM_SETUP:
        std::cout << "setup BoidCPU";
        break;
    case MODE_CALC_NBRS:
        std::cout << "calculate neighbours";
        break;
    case CMD_NBR_REPLY:
        std::cout << "neighbouring boids from neighbour";
        break;
    case MODE_POS_BOIDS:
        std::cout << "calculate new boid positions";
        break;
    case MODE_LOAD_BAL:
        std::cout << "load balance";
        break;
    case CMD_LOAD_BAL:
        std::cout << "load balance instructions";
        break;
    case CMD_LOAD_BAL_REQUEST:
        std::cout << "load balance request";
        break;
    case CMD_BOUNDS_AT_MIN:
        std::cout << "BoidCPU at minimal bounds";
        break;
    case MODE_TRAN_BOIDS:
        std::cout << "transfer boids";
        break;
    case CMD_BOID:
        std::cout << "boid in transit";
        break;
    case MODE_DRAW:
        std::cout << "send boids to BoidGPU";
        break;
    case CMD_DRAW_INFO:
        std::cout << "boid info heading to BoidGPU";
        break;
    case CMD_ACK:
        std::cout << "ACK signal";
        break;
    case CMD_KILL:
        std::cout << "kill simulation";
        break;
    default:
        std::cout << "UNKNOWN COMMAND";
        break;
    }
    std::cout << std::endl;

    std::cout << "\t";
    printCommandLoop: for (int i = 0; i < CMD_HEADER_LEN; i++) {
        std::cout << data[i] << " ";
    }

    std::cout << "|| ";

    printCmdDataLoop: for (int i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
        std::cout << data[CMD_HEADER_LEN + i] << " ";
    }
    std::cout << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Classes /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//==============================================================================
// Boid Class ==================================================================
//==============================================================================

/******************************************************************************/
/*
 * The constructor for the Boid class used when no initial values for the boid 
 * parameters are supplied. In this situation, all parameters are initialised 
 * to 0.
 *
 * @param   None
 *
 ******************************************************************************/
Boid::Boid() {
    id = 0;

    position = Vector(0, 0);
    velocity = Vector(0, 0);

    boidNeighbourIndex = 0;
    boidNeighbourCount = 0;
}

/******************************************************************************/
/*
 * The constructor for the Boid class used when initial values for the boid 
 * parameters are supplied. 
 *
 * @param   _boidID         The ID of the boid
 * @param   initPosition    The (absolute) initial position of the boid
 * @param   initVelocity    The initial velocity of the boid
 *
 ******************************************************************************/
Boid::Boid(uint16 _boidID, Vector initPosition, Vector initVelocity) {
    id = _boidID;

    position = initPosition;
    velocity = initVelocity;

    boidNeighbourIndex = 0;
    boidNeighbourCount = 0;

    std::cout << "Created boid #" << id << std::endl;
    printBoidInfo();
}

/******************************************************************************/
/*
 * Updates the position of the current boid by applying the alignment, cohesion 
 * and separation behaviours defined by Craig Reynolds. 
 * 
 * The actual implementation used is based examples in 'The Nature of Code' by 
 * Daniel Shiffman: http://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void Boid::update(void) {
    std::cout << "Updating boid #" << id << std::endl;

    if (boidNeighbourCount > 0) {
        acceleration.add(separate());
        acceleration.add(align());
        acceleration.add(cohesion());
    }

    velocity.add(acceleration);

#ifdef REDUCED_LUT_USAGE
    int32_fp mag = velocity.mag();
    if (mag > MAX_VELOCITY) {
        velocity.setMag(MAX_VELOCITY);
    }
#else
    velocity.limit(MAX_VELOCITY);
#endif

    position.add(velocity);
    acceleration.mul(0);
    printBoidInfo();
}

/******************************************************************************/
/*
 * The alignment behaviour of boids. Leads to boids pointing in the same 
 * direction as their neighbouring boids. 
 * 
 * If the logic of the implementation needs reducing the resulting steering 
 * vector is not limited in terms of turning force. This causes the boids to 
 * turn instantaneously rather than gradually.  
 * 
 * Based examples in 'The Nature of Code' by Daniel Shiffman: 
 *  http://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 * @param   None
 *
 * @return          A steering vector indicating the change needed to align
 *
 ******************************************************************************/
Vector Boid::align(void) {
    Vector total;

    alignBoidsLoop: for (int i = 0; i < boidNeighbourCount; i++) {
        total.add(boidNeighbourList[boidNeighbourIndex][i]->velocity);
    }

    total.div(boidNeighbourCount);
    total.setMag(MAX_VELOCITY);
    Vector steer = Vector::sub(total, velocity);

#ifndef REDUCED_LUT_USAGE
    steer.limit(MAX_FORCE);
#endif

    return steer;
}

/******************************************************************************/
/*
 * The separation behaviour of boids. Leads to boids moving away from one 
 * another if they are too close to any neighbouring boids. 
 * 
 * If the logic of the implementation needs reducing the resulting steering 
 * vector is not limited in terms of turning force. This causes the boids to 
 * turn instantaneously rather than gradually.  
 * 
 * Based examples in 'The Nature of Code' by Daniel Shiffman: 
 *  http://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 * @param   None
 *
 * @return          A steering vector indicating the change needed to separate
 *
 ******************************************************************************/
Vector Boid::separate(void) {
    Vector total;
    Vector diff;

    separateBoidsLoop: for (int i = 0; i < boidNeighbourCount; i++) {
        diff = Vector::sub(position, boidNeighbourList[boidNeighbourIndex][i]->position);
        diff.normalise();
        total.add(diff);
    }

    total.div(boidNeighbourCount);
    total.setMag(MAX_VELOCITY);
    Vector steer = Vector::sub(total, velocity);

#ifndef REDUCED_LUT_USAGE
    steer.limit(MAX_FORCE);
#endif

    return steer;
}

/******************************************************************************/
/*
 * The cohesion behaviour of boids. Leads to boids grouping by making each boid 
 * move towards the centre of mass of its neighbouring boids. 
 * 
 * If the logic of the implementation needs reducing the resulting steering 
 * vector is not limited in terms of turning force. This causes the boids to 
 * turn instantaneously rather than gradually.  
 * 
 * Based examples in 'The Nature of Code' by Daniel Shiffman: 
 *  http://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 * @param   None
 *
 * @return          A steering vector indicating the change needed to cohese
 *
 ******************************************************************************/
Vector Boid::cohesion(void) {
    Vector total;

    coheseBoidLoop: for (int i = 0; i < boidNeighbourCount; i++) {
        total.add(boidNeighbourList[boidNeighbourIndex][i]->position);
    }

    total.div(boidNeighbourCount);
    Vector desired = Vector::sub(total, position);
    desired.setMag(MAX_VELOCITY);
    Vector steer = Vector::sub(desired, velocity);

#ifndef REDUCED_LUT_USAGE
    steer.limit(MAX_FORCE);
#endif
    return steer;
}

/******************************************************************************/
/*
 * Set the neighbour details for the current boid. Used when the neighbours of 
 * a boid are being calculated (which is done every simulation step). 
 * 
 * It was not possible for a Boid instance to contain a list of its neighbours. 
 * Therefore, each BoidCPU contains a list of neighbours for each boid it 
 * contains. Each boid needs to know at what index its neighbours are held in 
 * this structure and how many neighbours it has (to avoid reading beyond the 
 * edge of the arrary). This method is used to supply the current boid with 
 * that information. 
 *
 * @param   neighbourIndex  The index of the current boid's neighbours in the 
 *                          parent BoiCPU's neighbour data structure
 * @param   neighbourCount  The number of neighbours the current boid has
 *
 * @return  None
 *
 ******************************************************************************/
void Boid::setNeighbourDetails(uint8 neighbourIndex, uint8 neighbourCount) {
    boidNeighbourIndex = neighbourIndex;
    boidNeighbourCount = neighbourCount;
}

/******************************************************************************/
/*
 * Print out the state of the current boid to standard output. Used during 
 * debugging, not possible when synthesised to FPGA core.
 *
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void Boid::printBoidInfo() {
    std::cout << "==========Info for Boid " << id << "==========" << std::endl;
    std::cout << "Boid Position: [" << position.x << " " << position.y << "]"
            << std::endl;
    std::cout << "Boid Velocity: [" << velocity.x << " " << velocity.y << "]"
            << std::endl;
    std::cout << "===================================" << std::endl;
}

//==============================================================================
// Vector Class ================================================================
//==============================================================================

/******************************************************************************/
/*
 * Constructor for a vector when no initial values for the vector components 
 * are supplied. Here, the components are initialised to 0. 
 *
 * @param   None
 *
 ******************************************************************************/
Vector::Vector() {
    x = 0;
    y = 0;
}

/******************************************************************************/
/*
 * Constructor for a 2-dimensional vector when the initial values for the vector 
 * components are supplied. 
 *
 * @param   x_  The initial x-value of the vector
 * @param   y_  The initial y-value of the vector
 *
 ******************************************************************************/
Vector::Vector(int16_fp x_, int16_fp y_) {
    x = x_;
    y = y_;
}

/******************************************************************************/
/*
 * Adds the supplied value to the current vector. 
 *
 * @param   n   The value to add to the current vector
 *
 * @return  None
 *
 ******************************************************************************/
void Vector::add(Vector v) {
    x = x + v.x;
    y = y + v.y;
}

/******************************************************************************/
/*
 * Multiplies the current vector by the supplied value. Multiplication in 
 * hardware can be expensive, but is used here as it is less expensive when 
 * using fixed point values. 
 *
 * @param   n   The value to multiply the current vector by
 *
 * @return  None
 *
 ******************************************************************************/
void Vector::mul(int16_fp n) {
    x = x * n;
    y = y * n;
}

/******************************************************************************/
/*
 * Divides the current vector by the supplied value. Ensure that the input is 
 * always positive. Division in hardware is typically expensive, but is used 
 * here as it is less expensive when using fixed point values. 
 *
 * @param   n   The value to divide the current vector by
 *
 * @return  None
 *
 ******************************************************************************/
void Vector::div(int16_fp n) {
    x = x / n;
    y = y / n;
}

/******************************************************************************/
/*
 * Subtracts two vectors and returns the result.
 *
 * @param   v1  One of the vectors to subtract
 * @param   v2  The other vector to subtract
 *
 * @return      The difference between the two input vectors
 *
 ******************************************************************************/
Vector Vector::sub(Vector v1, Vector v2) {
    return Vector(v1.x - v2.x, v1.y - v2.y);
}

/******************************************************************************/
/*
 * Calculates the squared distance between two vectors - used to avoid use of
 * doubles and square roots, which are expensive in hardware.
 *
 * @param   v1  One of the vectors to determine the distance between
 * @param   v2  The other vector to determine the distance between
 *
 * @return      The squared distance between the two input vectors
 *
 ******************************************************************************/
int32_fp Vector::squaredDistanceBetween(Vector v1, Vector v2) {
    int32_fp xPart = v1.x - v2.x;
    int32_fp yPart = v1.y - v2.y;

    return (xPart*xPart) + (yPart*yPart);
}

/******************************************************************************/
/*
 * Calculate the magnitude of the current vector. Typically uses the square 
 * root operation, which is expensive in hardware. Here, a fixed point square 
 * root operation is used, which is less expensive than an integer version. 
 *
 * @param   None
 *
 * @return      The magnitude of the current vector. 
 *
 ******************************************************************************/
int16_fp Vector::mag() {
    int32_fp result = (x*x + y*y);
    return hls::sqrt(result);
}

/******************************************************************************/
/*
 * Set the magnitude (length) of the current vector to the supplied value.
 *
 * @param   newMag  The value to set the magnitude of the vector to
 *
 * @return  None
 *
 ******************************************************************************/
void Vector::setMag(int16_fp newMag) {
    normalise();
    mul(newMag);
}

/******************************************************************************/
/*
 * Limit the length of a vector to the specified value. If the length of the 
 * vector is less than this value, the vector remains unchanged. 
 *
 * @param   max The value to limit the vector to
 *
 * @return  None
 *
 ******************************************************************************/
#ifndef REDUCED_LUT_USAGE
void Vector::limit(int16_fp max) {
    int16_fp m = mag();
    if (m > max) {
        setMag(max);
    }
}
#endif

/******************************************************************************/
/*
 * Normalises the current vector. This creates a unit vector (length of 1) that  
 * has the same direction as the original vector. 
 *
 * @TODO    If the vector is of length 0, leave the vector unchanged
 * 
 * @param   None
 *
 * @return  None
 *
 ******************************************************************************/
void Vector::normalise() {
    int16_fp magnitude = mag();

    if (magnitude != 0) {
        div(magnitude);
    } else {
        x = 0;
        y = 0;
    }
}
