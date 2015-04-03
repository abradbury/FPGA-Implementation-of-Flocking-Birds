#include "boidMaster.h"

// Global variables ============================================================
uint32 tbOutputData[20][MAX_CMD_LEN];
uint32 tbInputData[MAX_INPUT_CMDS][MAX_CMD_LEN];
uint32 tbOutputCount = 0;
uint32 tbInputCount = 0;

uint32 tbData[MAX_CMD_BODY_LEN];
uint32 tbTo;
uint32 tbFrom;
uint32 tbDataLength = 0;

uint32 tbGatekeeperCount;
uint32 tbGatekeeperIDs[8];

// Function headers ============================================================
void simulateAck(uint32 from);

void simulatePingStart();
void simulateUserInfo();
void issueEndOfPing();
void simulatePingReplies();
void simulateSetupAck();
void simulateNbrSearchAck();
void simulatePositionBoidsAck();
void simulateBoidTransferAck();
void simulateBoidGPUAck();

void processSetupInfo();
void processDrawMode();


void tbPrintCommand(bool send, uint32 *data);
void tbCreateCommand(uint32 len, uint32 to, uint32 from, uint32 type,
		uint32 *data);


int main() {
    hls::stream<uint32> to_hw, from_hw;

    // Test BoidMaster input ---------------------------------------------------
    simulatePingStart();
    simulatePingReplies();
    issueEndOfPing();

    simulateUserInfo();
    simulateSetupAck();

    simulateNbrSearchAck();
    simulatePositionBoidsAck();
    simulateBoidTransferAck();

    simulateBoidGPUAck();

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
		switch (tbInputData[tbInputCount][CMD_TYPE]) {
//			case CMD_PING:
//				processPing();
//				break;
			case CMD_SIM_SETUP:
				processSetupInfo();
				break;
//			case MODE_CALC_NBRS:
//				processCalcNeighbours();
//				break;
//			case MODE_POS_BOIDS:
//				processBoidPositioning();
//				break;
//			case MODE_TRAN_BOIDS:
//				processBoidTransfer();
//				break;
			case MODE_DRAW:
				processDrawMode();
				break;
			default:
				break;
		}

		// Check for more input ------------------------------------------------
		// Don't really need input buffer if data is processed before the next
		// inputCount++;
		inputAvailable = from_hw.read_nb(tbInputData[tbInputCount][CMD_LEN]);
	}

	std::cout << "=====TestBench finished receiving=====" << std::endl;

    return 0;	// A non-zero return value signals an error
}

//============================================================================//
// State processing functions ------------------------------------------------//
//============================================================================//

// 4 1 [from] 17 ||
void simulateAck(uint32 from) {
	tbDataLength = 0;
	tbTo = CONTROLLER_ID;
	tbFrom = from;			// Just some random ID
	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_ACK, tbData);
}

void simulatePingStart() {
	std::cout << "Simulating ping start..." << std::endl;
	tbDataLength = 0;
	tbTo = CONTROLLER_ID;
	tbFrom = 1481765933;
	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_START, tbData);
}

// 5 1 2 4 || 60
void simulateUserInfo() {
	std::cout << "Simulating user info..." << std::endl;
	tbData[0] = 20;			// Number of boids
	tbDataLength = 1;
	tbTo = CONTROLLER_ID;
	tbFrom = 1481765933;
	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_USER_INFO, tbData);
}

// 5 1 42 3 || 2
// 5 1 66 3 || 4
void simulatePingReplies() {
	std::cout << "Simulating ping replies..." << std::endl;
	tbGatekeeperCount = 1;
	tbGatekeeperIDs[0] = 1481765933;
//	tbGatekeeperIDs[1] = 66;

	tbData[0] = 2;					// Number of resident BoidCPUs
	tbDataLength = 1;
	tbTo = CONTROLLER_ID;
	tbFrom = tbGatekeeperIDs[0];	// Random Gatekeeper ID
	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_REPLY, tbData);

//	tbData[0] = 4;					// Number of resident BoidCPUs
//	tbDataLength = 1;
//	tbTo = CONTROLLER_ID;
//	tbFrom = tbGatekeeperIDs[1];	// Random Gatekeeper ID
//	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_REPLY, tbData);

	std::cout << "Responding to ping with 6 BoidCPUs (2/4)..." << std::endl;
}

void issueEndOfPing() {
	std::cout << "Simulating end of ping..." << std::endl;
	tbDataLength = 0;
	tbTo = CONTROLLER_ID;
	tbFrom = 1481765933;
	tbCreateCommand(tbDataLength, tbTo, tbFrom, CMD_PING_END, tbData);
}

void processSetupInfo() {
	std::cout << "Processing setup info..." << std::endl;

	// Store BoidCPU information
	uint32 boidCPUID = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_NEWID_IDX];
	uint32 boidCount = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_BDCNT_IDX];
	uint32 distNbrs	 = tbInputData[0][CMD_HEADER_LEN + CMD_SETUP_NBCNT_IDX];
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

void simulateSetupAck() {
	std::cout << "Simulating setup ACK..." << std::endl;
	for (int i = 0; i < tbGatekeeperCount; i++) {
		simulateAck(tbGatekeeperIDs[i]);
	}
}

void simulateNbrSearchAck() {
	std::cout << "Simulating neighbour search ACK..." << std::endl;
	for (int i = 0; i < tbGatekeeperCount; i++) {
		simulateAck(tbGatekeeperIDs[i]);
	}
}

void simulatePositionBoidsAck() {
	std::cout << "Simulating position boids ACK..." << std::endl;
	for (int i = 0; i < tbGatekeeperCount; i++) {
		simulateAck(tbGatekeeperIDs[i]);
	}
}

void simulateBoidTransferAck() {
	std::cout << "Simulating boid transfer ACK..." << std::endl;
	for (int i = 0; i < tbGatekeeperCount; i++) {
		simulateAck(tbGatekeeperIDs[i]);
	}
}

void simulateBoidGPUAck() {
	std::cout << "Simulating BoidGPU ACK..." << std::endl;
	simulateAck(BOIDGPU_ID);
}

void processDrawMode() {
	// TODO
}

//============================================================================//
// Message processing functions ----------------------------------------------//
//============================================================================//

void tbCreateCommand(uint32 len, uint32 to, uint32 from, uint32 type,
		uint32 *data) {
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
}

//============================================================================//
// Debug ---------------------------------------------------------------------//
//============================================================================//

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
	case CMD_LOAD_BAL:
		std::cout << "load balance                       ";
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
