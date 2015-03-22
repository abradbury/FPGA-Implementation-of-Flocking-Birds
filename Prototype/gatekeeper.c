/**
 * Routes data between internal BoidCPUs, filters out irrelevant external data
 * and forwards internal data to external entities as appropriate.
 */

//TODO: Have the Gatekeeper intercept the setup commands so that it knows the
// IDs of the BoidCPUs that it contains and the neighbours of those BoidCPUs

#include "xparameters.h"	// Xilinx definitions
#include "xemaclite.h"		// Ethernet
#include "fsl.h"        	// AXI Steam
#include "boids.h"			// Boid definitions

// Setup Ethernet MAC addresses and transmit and receive buffers
XEmacLite ether;

// TODO: Determine maximum buffer size
// TODO: Ethernet requires a u8 buffer, but FXL requires a u32...
u8 tmit_buffer[XEL_MAX_FRAME_SIZE];
u8 recv_buffer[XEL_MAX_FRAME_SIZE];

u8 residentBoidCPUCount = 0;
u8 residentBoidCPUChannels[MAX_BOIDCPUS_PER_FPGA];
u8 residentBoidCPUNeighbours[MAX_BOIDCPU_NEIGHBOURS * MAX_BOIDCPUS_PER_FPGA];

// Protocols
u8 channelLookup(u32 to);
bool arrivalCheckPassed(u32 from);
bool departureCheckPassed(u32 to);

void putFSLData(u32 value, u32 channel);
u32 getFSLData(u32 *data, u32 channel);

void processExternalMessage();
void processInternalMessage(u32 *data, u8 channel);
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data);

// TODO: Rename to main() when deployed
int mainThree() {

	// Set up the Ethernet
	XEmacLite_Config *etherconfig = XEmacLite_LookupConfig(
			XPAR_EMACLITE_0_DEVICE_ID);
	XEmacLite_CfgInitialize(&ether, etherconfig, etherconfig->BaseAddress);

	// Main execution loop
	do {
		// Check for external (Ethernet) data ----------------------------------
		volatile int recv_len = 0;
		recv_len = XEmacLite_Recv(&ether, recv_buffer);

		// Process received external data --------------------------------------
		if (recv_len != 0) {
			processExternalMessage();
		}

		// Check for internal (FXL/AXI) data -----------------------------------
		u32 channelZeroData[MAX_CMD_LEN];
		int channelZeroInvalid = getFSLData(channelZeroData, 0);

		u32 channelOneData[MAX_CMD_LEN];
		int channelOneInvalid = getFSLData(channelOneData, 1);

		// Process received internal data --------------------------------------
		if (!channelZeroInvalid) {
			processInternalMessage(channelZeroData, 0);
		}

		if (!channelOneInvalid) {
			processInternalMessage(channelOneData, 1);
		}

	} while (1);

	return 0;
}

/**
 * Sends a message to an external entity beyond the FPGA that the MicroBlaze
 * resides on. Ethernet is used to transmit the messages to other FPGAs.
 */
void sendExternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
	// First, create the message
	u32 command[MAX_CMD_LEN];
	int i = 0;

	command[CMD_LEN] = len + CMD_HEADER_LEN;
	command[CMD_TO] = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		for (i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}

	// Then, populate the transmit buffer
	for (i = 0; i < command[CMD_LEN]; i++) {
		tmit_buffer[i] = command[i];
	}

	// Finally, clear the receive buffer before sending
	XEmacLite_FlushReceive(&ether);
	XEmacLite_Send(&ether, tmit_buffer, command[CMD_LEN]);
}

/**
 * Sends a message internally, over the FSL/AXI bus/stream, to BoidCPUs that
 * reside on the same FPGA that the MicroBlaze does.
 */
void sendInternalMessage(u32 len, u32 to, u32 from, u32 type, u32 *data) {
	// First, create the message
	u32 command[MAX_CMD_LEN];
	int i = 0;

	command[CMD_LEN] = len + CMD_HEADER_LEN;
	command[CMD_TO] = to;
	command[CMD_FROM] = from;
	command[CMD_TYPE] = type;

	if (len > 0) {
		for (i = 0; i < len; i++) {
			command[CMD_HEADER_LEN + i] = data[i];
		}
	}

	// Then, check which channel the recipient resides on
	int channel = channelLookup(to);

	// Finally, send the message
	for (i = 0; i < CMD_HEADER_LEN + len; i++) {
		if (channel == 0) {
			putFSLData(command[i], 0);
		} else if (channel == 1) {
			putFSLData(command[i], 1);
		} else {
			putFSLData(command[i], 0);
			putFSLData(command[i], 1);
		}
	}
}

/**
 * On receiving an external message, determine if it is addressed to any
 * internal BoidCPUs. If it is, then forward it internally, if not, ignore it.
 */
void processExternalMessage() {
	// Check if the message is applicable to contained BoidCPUs
	if(arrivalCheckPassed(recv_buffer[CMD_FROM])) {
		// Collate the data to forward
		int dataLength = recv_buffer[CMD_LEN] - CMD_HEADER_LEN;
		u32 dataToForward[dataLength];

		int i = 0;
		for(i = 0; i < dataLength; i++) {
			dataToForward[i] = recv_buffer[CMD_HEADER_LEN + i];
		}

		sendInternalMessage(recv_buffer[CMD_LEN], recv_buffer[CMD_TO],
			recv_buffer[CMD_FROM], recv_buffer[CMD_TYPE], dataToForward);
	}
}

/**
 * On receiving an internal message, determine if the message recipient is
 * internal or external and forward as appropriate.
 */
void processInternalMessage(u32 *data, u8 channel) {
	// Collate the data to forward
	int dataLength = data[CMD_LEN] - CMD_HEADER_LEN;
	u32 dataToForward[dataLength];

	int i = 0;
	for(i = 0; i < dataLength; i++) {
		dataToForward[i] = data[CMD_HEADER_LEN + i];
	}

	// If the command is a multicast message, send it both internally and
	// externally. Otherwise, check if the message recipient is external.
	//
	// TODO: Improve this so that a check is made as to whether all the
	// 	neighbours are within the current BoidCPU.
	if (data[CMD_TO] == CMD_MULTICAST) {
		sendInternalMessage(data[CMD_LEN], data[CMD_TO], data[CMD_FROM],
			data[CMD_TYPE], dataToForward);

		sendExternalMessage(data[CMD_LEN], data[CMD_TO], data[CMD_FROM],
			data[CMD_TYPE], dataToForward);
	} else if(departureCheckPassed(data[CMD_TO])) {
		sendExternalMessage(data[CMD_LEN], data[CMD_TO], data[CMD_FROM],
			data[CMD_TYPE], dataToForward);
	}
}

/**
 * Determines if external messages should be forwarded internally. Returns true
 * when a message should be forwarded internally, false if it should not.
 *
 * Allow the external message in if:
 *  - It is from the controller (broadcast or direct)
 *  - It is from a neighbour of one of the resident BoidCPUs
 */
bool arrivalCheckPassed(u32 from) {
	if (from == CONTROLLER_ID) {
		return true;
	} else if (from >= FIRST_BOIDCPU_ID) {
		int i = 0;
		for (i = 0; i < residentBoidCPUCount; i++) {
			if (from == residentBoidCPUNeighbours[i]) {
				return true;
			}
		}
	}

	return false;
}

/**
 * Determine if an internal message should be forwarded externally. Return true
 * if it should and false if it should not be.
 *
 * Forward the message externally if:
 *  - It is to the controller
 *  - It is to the BoidGPU
 */
bool departureCheckPassed(u32 to) {
	if ((to == CONTROLLER_ID) || (to == BOIDGPU_ID)) {
		return true;
	} else {
		return false;
	}
}

u8 channelLookup(u32 to) {
	// TODO: Finish channel lookup implementation

	return 0;
}

/**
 * A wrapper for the getfsxl() method. Checks if there is any data on the
 * specified channel and reads it all in if there is. Notifies of errors.
 */
u32 getFSLData(u32 *data, u32 channel) {
	int invalid, error, value = 0;

	if (channel == 1) getfslx(value, 1, FSL_NONBLOCKING);
	else if (channel == 0) getfslx(value, 0, FSL_NONBLOCKING);

	fsl_isinvalid(invalid);          	// Was there any data?
	fsl_iserror(error);					// Was there an error?

	if (error) {
		xil_printf("Error receiving data on Channel %d: %d\n\r", channel, error);
	}

	if (!invalid) {
		data[CMD_LEN] = value;
		xil_printf("Received data (Channel %d)\n\r", channel);
		int i = 0;

		// Handle invalid length values
		if ((data[CMD_LEN] == 0) || (data[CMD_LEN] > MAX_CMD_LEN)) {
			data[CMD_LEN] = MAX_CMD_LEN;
			print("Message has invalid length - correcting\n\r");
		}

		for (i = 0; i < data[CMD_LEN] - 1; i++) {
			if (channel == 1) getfslx(value, 1, FSL_NONBLOCKING);
			else if (channel == 0) getfslx(value, 0, FSL_NONBLOCKING);

			fsl_iserror(error);				// Was there an error?
			if (error) {
				xil_printf("Error receiving data on Channel %d: %d\n\r", channel, error);
			}

			data[i + 1] = value;
		}
	}

	return invalid;
}

/**
 * A wrapper for putting data onto the (internal) FSL bus. Can catch write
 * errors and alert when the buffer is full.
 */
void putFSLData(u32 value, u32 channel) {
	int error = 0, invalid = 0;

	// TODO: Ensure that writes use FSL_DEFAULT (blocking write)
	if (channel == 1) putfslx(value, 1, FSL_DEFAULT);
	else if (channel == 0) putfslx(value, 0, FSL_DEFAULT);

	fsl_isinvalid(invalid);
	fsl_iserror(error);

	if (invalid) {
		xil_printf("Warning - channel %d is full: %d\n\r", channel, value);
	}

	if (error) {
		xil_printf("Error writing data to channel %d: %d\n\r", channel, value);
	}
}

