#include "boidCPU.h"

// Globals
uint32 tbOutputData[20][MAX_CMD_LEN];
uint32 tbInputData[MAX_INPUT_CMDS][MAX_CMD_LEN];
uint32 tbOutputCount = 0;
uint32 tbInputCount = 0;

uint32 data[MAX_CMD_BODY_LEN];
uint32 to;
uint32 from = CONTROLLER_ID;
uint32 dataLength = 0;

uint32 coords[EDGE_COUNT];
uint32 neighbours[MAX_BOIDCPU_NEIGHBOURS];

bool drawBoids = false;

// Function headers
void testPing();
void testSimulationSetup();
void testNeighbourSearch();
void testNeighbourResponse();
void testCalcNextBoidPos();
void testLoadBalance();
void testMoveBoids();
void testDrawBoids();

void processPingResponse();
void processNeighbourReply();
void processDrawInfo();

void tbPrintCommand(bool send, uint32 *data);
void createCommand(uint32 len, uint32 to, uint32 from, uint32 type, uint32 *data);

/**
 * This acts as the external interface to the BoidCPU
 */
int main() {
    hls::stream<uint32> to_hw, from_hw;

    // Test BoidCPU input ------------------------------------------------------
    // First send the initialisation commands
    testPing();
    testSimulationSetup();

    // Then draw the initial positions of the boids
    testDrawBoids();

    // Then repeat these commands every time step
    testNeighbourSearch();
    testNeighbourResponse();
    testCalcNextBoidPos();
    testLoadBalance();
    testMoveBoids();
    testDrawBoids();

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
    toplevel(to_hw, from_hw);

    // Receive data ------------------------------------------------------------
    bool inputAvailable = from_hw.read_nb(tbInputData[tbInputCount][CMD_LEN]);

    while (inputAvailable) {
        inLp: for (int i = 0; i < tbInputData[tbInputCount][CMD_LEN] - 1; i++) {
            tbInputData[tbInputCount][i + 1] = from_hw.read();
        }

        tbPrintCommand(false, tbInputData[tbInputCount]);

        // Process data --------------------------------------------------------
        switch (tbInputData[tbInputCount][CMD_TYPE]) {
            case CMD_PING_REPLY:
                processPingResponse();
                break;
            case CMD_NBR_REPLY:
                processNeighbourReply();
                break;
            case CMD_DRAW_INFO:
                processDrawInfo();
                break;
            default:
                std::cout << "Controller received a command it cannot handle: "
                    << tbInputData[CMD_TYPE] << std::endl;
                break;
        }

        // Check for more input ------------------------------------------------
        // Don't really need input buffer if data is processed before the next
        // inputCount++;
        inputAvailable = from_hw.read_nb(tbInputData[tbInputCount][CMD_LEN]);
    }

    std::cout << "=====TestBench finished receiving=====" << std::endl;

    return 0;
}

void testPing() {
    // Test ping response ----------------------------------------------------//
    // 4, 0, 1, 2 ||
    to = CMD_BROADCAST;
    dataLength = 0;
    createCommand(dataLength, to, from, CMD_PING, data);
}

// TODO: Need to send to broadcast during actual testing as random ID unknown
void testSimulationSetup() {
    // Test simulation setup ---------------------------------------------------
    // 18, 83, 1, 5 || 7, 10, 0, 0, 40, 40, 3, 4, 5, 8, 11, 10, 9, 6, [100]
    dataLength = 14;
    to = 69;            // The current random ID of the test BoidCPU

    uint32 newID = 7;
    uint32 initialBoidCount = 10;
    coords[0] = 0;
    coords[1] = 0;
    coords[2] = 40;
    coords[3] = 40;

    neighbours[0] = 3;
    neighbours[1] = 4;
    neighbours[2] = 5;
    neighbours[3] = 8;
    neighbours[4] = 11;
    neighbours[5] = 10;
    neighbours[6] = 9;
    neighbours[7] = 6;

    data[0] = newID;
    data[1] = initialBoidCount;

    for (int i = 0; i < EDGE_COUNT; i++) {
        data[2 + i] = coords[i];
    }

    for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
        data[EDGE_COUNT + 2 + i] = neighbours[i];
    }

    // Additional data for testing
    // If the value is not 0 then the BoidCPU is able to progress itself until
    // it reaches the time step equal to the supplied value - it does not need
    // to wait for the controller to supply synchronisation steps
//  data[14] = 100;
//  dataLength += 1;

    createCommand(dataLength, to, from, CMD_SIM_SETUP, data);
}

void testNeighbourSearch() {
    // 4 0 1 6 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_CALC_NBRS, data);
}

/**
 * Simulates responses from neighbouring BoidCPUs to the BoidCPU under test
 * when the system is in the neighbour search mode.
 */
void testNeighbourResponse() {
	// For each neighbouring BoidCPU, create a list of boids and send this to
	// the BoidCPU under test
	// FIXME: This seems to be called before all other testbench stuff
	const int boidsPerBoidCPU = 10;
	const int positionBounds[4] = {0, 0, 720, 720};
	Boid boidsFromNbrs[MAX_BOIDCPU_NEIGHBOURS][boidsPerBoidCPU];

	for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		for (int j = 0; j < boidsPerBoidCPU; j++) {
			Vector vel = Vector(-MAX_VELOCITY + (rand() % (int)(MAX_VELOCITY -
				-MAX_VELOCITY + 1)), -MAX_VELOCITY + (rand() % (int)
				(MAX_VELOCITY - -MAX_VELOCITY + 1)));

			Vector pos = Vector(positionBounds[0] + (rand() % (int)
				(positionBounds[2] - positionBounds[0] + 1)),
				positionBounds[1] + (rand() % (int)(positionBounds[3] -
				positionBounds[1] + 1)));

			int boidID = ((neighbours[i] - 1) * boidsPerBoidCPU) + j + 1;

			boidsFromNbrs[i][j] = Boid(boidID, pos, vel, j);
		}
	}

	// Now send the data to the BoidCPU under test
	for (int i = 0; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		// The next step is to create the message data
		for (int j = 0, k = 0; j < boidsPerBoidCPU; j++, k++) {
			uint32 position = 0;
			uint32 velocity = 0;

			position |= ((uint32)(boidsFromNbrs[i][j].position.x) << 20);
			position |= ((uint32)(boidsFromNbrs[i][j].position.y) << 8);

			// Despite being of type int12, the velocity (and position) seem to
			// be represented using 16 bits. Therefore, negative values need to
			// have bits 12 to 15 set to 0 (from 1) before ORing with velocity.
			if (boidsFromNbrs[i][j].velocity.x < 0) {
				velocity |= ((uint32)((boidsFromNbrs[i][j].velocity.x) & ~((int16)0x0F << 12)) << 20);
			} else {
				velocity |= ((uint32)(boidsFromNbrs[i][j].velocity.x) << 20);
			}

			if (boidsFromNbrs[i][j].velocity.y < 0) {
				velocity |= ((uint32)((boidsFromNbrs[i][j].velocity.y) & ~((int16)0x0F << 12)) << 8);
			} else {
				velocity |= ((uint32)(boidsFromNbrs[i][j].velocity.y) << 8);
			}

			data[(k * BOID_DATA_LENGTH) + 0] = position;
			data[(k * BOID_DATA_LENGTH) + 1] = velocity;
			data[(k * BOID_DATA_LENGTH) + 2] = boidsFromNbrs[i][j].id;
		}

		int dataLength = boidsPerBoidCPU * BOID_DATA_LENGTH;
		createCommand(dataLength, 7, neighbours[i], CMD_NBR_REPLY, data);
	}
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

void testDrawBoids() {
    // 4 0 1 14 ||
    dataLength = 0;
    to = CMD_BROADCAST;
    createCommand(dataLength, to, from, MODE_DRAW, data);
}

void processPingResponse() {
    // TODO
}

/**
 * Create a list of neighbouring boids from the boids sent by a neighbouring
 * BoidCPU. Used when calculating the neighbours for a particular boid.
 */
void processNeighbourReply() {
    int count = (tbInputData[tbInputCount][CMD_LEN] - CMD_HEADER_LEN) / 
        BOID_DATA_LENGTH;
    Boid tbBoids[MAX_BOIDS];

    std::cout << "Dummy BoidCPU received " << count << " boids" << std::endl;

    for (int i = 0; i < count; i++) {
        uint32 position = tbInputData[tbInputCount][CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 0];
        uint32 velocity = tbInputData[tbInputCount][CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 1];

        Vector p = Vector((int12)((position & (~(uint32)0xFFFFF)) >> 20),
                (int12)((position & (uint32)0xFFF00) >> 8));

        Vector v = Vector((int12)((velocity & (~(uint32)0xFFFFF)) >> 20),
                (int12)((velocity & (uint32)0xFFF00) >> 8));

        Boid b = Boid((uint16)tbInputData[tbInputCount][CMD_HEADER_LEN + (BOID_DATA_LENGTH * i) + 2], p, v, i);
        tbBoids[i] = b;
//      b.printBoidInfo();
    }
}

void processDrawInfo() {
    std::cout << "Drawing boids..." << std::endl;

    if (drawBoids == true) {
        int maxBoidID = 0;
        int digits = 0;
        int widthDigits = 0;
        int idDigits = 0;
        int boidAtPos = 0;
        char space = '-';
        char edge = '*';

        // Get the boid bounds
        int boidCPUWidth =  coords[X_MAX] - coords[X_MIN];
        int boidCPUHeight = coords[Y_MAX] - coords[Y_MIN];

        // Get the number of boids
        int tbBoidCount = (tbInputData[tbInputCount][CMD_LEN] - CMD_HEADER_LEN) / 3;

        // Get the maximum boid ID
        for (int i = 0; i < tbBoidCount; i++) {
            if (tbInputData[tbInputCount][CMD_HEADER_LEN + (i * 3)] > maxBoidID) {
                maxBoidID = tbInputData[tbInputCount][CMD_HEADER_LEN + (i * 3)];
            }
        }

        // Determine the number of digits in the max boid ID
        if (maxBoidID < 10) {
            idDigits = 1;
        } else if (maxBoidID < 100) {
            idDigits = 2;
        } else {
            idDigits = 3;
        }

        // Calculate the top edge offset due to the left edge and index
        if (boidCPUHeight < 10) widthDigits = 1;
        else if (boidCPUHeight < 100) widthDigits = 2;
        else widthDigits = 3;
        std::cout << std::string(widthDigits, ' ') << edge;

        // Print the top index row
        for (int i = 0; i < boidCPUWidth; i++) {
            if (i < 10) digits = widthDigits;
            else if (i < 100) digits = widthDigits - 1;
            else digits = widthDigits - 2;
            std::cout << i << std::string(digits, ' ');
        } std::cout << std::endl;

        // Print the top edge (including offset)
        std::cout << std::string(widthDigits, ' ') << edge;
        for (int i = 0; i < boidCPUWidth; i++) {
            std::cout << edge << std::string(idDigits, ' ');
        } std::cout << std::endl;

        // Print the positions of the boids in the BoidCPU
        for (int y = 0; y < boidCPUHeight; y++) {
            // Print the left edge and index
            if (y < 10) digits = widthDigits - 1;
            else if (y < 100) digits = widthDigits - 2;
            else digits = widthDigits - 3;
            std::cout << std::string(digits, ' ') << y << edge;

            // Print the boid ID at the appropriate position
            for (int x = 0; x < boidCPUWidth; x++) {
                for (int i = 0, j = 0; j < tbBoidCount; j++, i = i + 3) {
                    int boidID = tbInputData[tbInputCount][CMD_HEADER_LEN + i];
                    int boidX =  tbInputData[tbInputCount][CMD_HEADER_LEN + i + 1];
                    int boidY =  tbInputData[tbInputCount][CMD_HEADER_LEN + i + 2];

                    if (boidX == x) {
                        if (boidY == y) {
                            boidAtPos = tbInputData[tbInputCount][CMD_HEADER_LEN + i];
                            break;
                        }
                    }
                }

                if (boidAtPos != 0) {
                    if (boidAtPos < 10) digits = idDigits;
                    else if (boidAtPos < 100) digits = idDigits - 1;
                    else digits = idDigits - 2;

                    std::cout << boidAtPos << std::string(digits, space);

                    boidAtPos = 0;
                } else {
                    std::cout << std::string(idDigits + 1, space);
                }
            }

            // Print the right edge
            std::cout << edge << y << std::endl;
        }

        // Print the bottom edge (including offset)
        std::cout << std::string(widthDigits, ' ') << edge;
        for (int i = 0; i < boidCPUWidth; i++) {
            std::cout << edge << std::string(idDigits, ' ');
        } std::cout << std::endl;

        // Print the bottom index row
        std::cout << std::string(widthDigits, ' ') << edge;
        for (int i = 0; i < boidCPUWidth; i++) {
            if (i < 10) digits = widthDigits;
            else if (i < 100) digits = widthDigits - 1;
            else digits = widthDigits - 2;
            std::cout << i << std::string(digits, ' ');
        } std::cout << std::endl;
    } else {
        // TODO: Print out boid info
//      int tbBoidCount = (tbInputData[tbInputCount][CMD_LEN] - CMD_HEADER_LEN) / 3;

        for (int i = 0; i < tbInputData[tbInputCount][CMD_LEN] - CMD_HEADER_LEN; i += 3) {
            std::cout << "Boid " << tbInputData[tbInputCount][CMD_HEADER_LEN + i + 0]
                << " has position [" << tbInputData[tbInputCount][CMD_HEADER_LEN + i + 1]
                << ", " << tbInputData[tbInputCount][CMD_HEADER_LEN + i + 2] << "]" << std::endl;
        }
    }
}

void createCommand(uint32 len, uint32 to, uint32 from, uint32 type, uint32 *data) {
    tbOutputData[tbOutputCount][CMD_LEN]  = len + CMD_HEADER_LEN;
    tbOutputData[tbOutputCount][CMD_TO]   = to;
    tbOutputData[tbOutputCount][CMD_FROM] = from;
    tbOutputData[tbOutputCount][CMD_TYPE] = type;

    if (len > 0) {
        dataToCmd: for (int i = 0; i < len; i++) {
            tbOutputData[tbOutputCount][CMD_HEADER_LEN + i] = data[i];
        }
    }

    tbOutputCount++;
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void tbPrintCommand(bool send, uint32 *data) {
    if (send) {
        if (data[CMD_TO] == CMD_BROADCAST) {
            std::cout << "-> TX, Controller sent broadcast: ";
        } else if (data[CMD_TO] == BOIDGPU_ID) {
            std::cout << "-> TX, Controller sent command to BoidGPU: ";
        } else {
            std::cout << "-> TX, Controller sent command to " << data[CMD_TO] << ": ";
        }
    } else {
        if (data[CMD_TO] == CMD_BROADCAST) {
            // This should never happen - BoidCPUs should not be able to broadcast
            std::cout << "<- RX, Controller received broadcast from " << data[CMD_FROM] << ": ";
        } else if (data[CMD_FROM] == BOIDGPU_ID) {
            // This should never happen
            std::cout << "<- RX, Controller received command from BoidGPU: ";
        } else {
            std::cout << "<- RX, Controller received command from " << data[CMD_FROM] << ": ";
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
        case CMD_LOAD_BAL:
            std::cout << "load balance";
            break;
        case MODE_TRAN_BOIDS:
            std::cout << "transfer boids";
            break;
        case CMD_BOID:
            std::cout << "boid";
            break;
        case MODE_DRAW:
            std::cout << "send boids to BoidGPU";
            break;
        case CMD_DRAW_INFO:
            std::cout << "boid info heading to BoidGPU";
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
    for (int i = 0; i < CMD_HEADER_LEN; i++) {
        std::cout << data[i] << " ";
    }

    std::cout << "|| ";

    for (int i = 0; i < data[CMD_LEN] - CMD_HEADER_LEN; i++) {
        std::cout << data[CMD_HEADER_LEN + i] << " ";
    }
    std::cout << std::endl;
}

