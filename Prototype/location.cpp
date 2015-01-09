#include "location.h"

#define SIZEOF_ARRAY( a ) (sizeof( a ) / sizeof( a[ 0 ] ))

#define CMD_HEADER_LEN	5	//
#define MAX_CMD_BODY_LEN 20	//
#define MAX_CMD_LEN		CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define MAX_NEIGHBOURS 	8	// The maximum number of neighbouring locations
#define MAX_LOCATIONS 	100	// The maximum number of location allowed

#define CMD_PING		1	// Controller asking how many locations their are
#define CMD_KILL		2	// Controller stopping the simulation
#define CMD_PING_REPLY	3	// Location response to controller ping
#define CMD_INIT		4	// Controller initiation command
#define CMD_BEGIN 		5	// Begin the simulation
#define CMD_LOAD_INFO	6	// Each location reports its current load
#define CMD_LOAD_ACT	7	// The decision of the controller based on the load
#define CMD_LOC_UPDATE	8	// The new parameters for location if load balanced
#define CMD_BOID		9	// Used to transfer boids between locations

#define BROADCAST		0	// Used by the controller to address all locations

#define VISIONRADIUS 3
#define MAXSPEED 5
#define MAXBOIDS 10

//// Class Headers ===============================================================
//class OldVector {
//	public:
//		int8 x;
//		int8 y;
//		int8 z;
//
//		OldVector();
//		OldVector(int8 x_, int8 y_, int8 z_);
//
//		void add(OldVector v);
//		void sub(OldVector v);
//		void mul(uint8 n);
//		void div(uint8 n);
//		uint8 mag();
//		void normalise();
//		void bound(uint8 n);
//		void rand(uint8 min, uint8 max);
//		bool empty();
//
//		// TODO: Is this needed?
//		friend std::ostream& operator<<(std::ostream& os, const OldVector& v);
//};
//
//class Boid {
//	OldVector position;
//	OldVector velocity;
//	uint8 id;
//	uint8 nCount;
//	uint8 neighbours[MAX_NEIGHBOURS];
//
//	public:
//		Boid(OldVector pos, OldVector vel, uint8 id);
//		uint8 getID ();
//		OldVector getVelocity();
//		OldVector getPosition();
//		uint8* getNeighbours();
//		uint8 getNeighbour(uint8 neighbourID);
//		uint8 getNeighbourCount();
//
//		void setVelocity(OldVector newVelocity);
//		void update(OldVector velocity);
//		void addNeighbour(uint8 neighbourID);
//		void resetNeighbours();
//
//		void draw();
//		void printBoidInfo();
//};
////==============================================================================


// Function Headers ============================================================
//void initialise(uint32 *data);
//
//void calcNextBoidPositions();
//
//void sendLoadInfo();
//void createCommand(uint32 *command, uint32 to, uint32 type, uint32 len, uint32 *data);
//void printCommand(uint32* command, bool send);
//
//void initialiseBoids(uint32 initBoidCount);
//
//void setupEnvironment(uint32 *data);
//void calcNeighbours(Boid* b);
////uint8 calcDistance(OldVector p1, OldVector p2);
//OldVector alignment(Boid* b);
//OldVector cohesion(Boid* b);
//OldVector separation(Boid* b);

// Parameter string
//Boid* boidList[MAXBOIDS];			// The indices correspond to the boid ID
//Boid* neighbours[2];				// FIXME: Change initial size
uint8 boidCount;

uint32 locationNeighbours[MAX_NEIGHBOURS];
uint32 locationPosition[8];

bool dbg;

uint8 locationID;
uint8 initBoidCount;
//==============================================================================


//// Classes =====================================================================
//// Constructors ////////////////////////////////////////////////////////////////
//OldVector::OldVector() {
//	x = 0;
//	y = 0;
//	z = 0;
//}
//
//OldVector::OldVector(int8 x_, int8 y_, int8 z_) {
//	x = x_;
//	y = y_;
//	z = z_;
//}
//
//// Basic Operations ////////////////////////////////////////////////////////////
//void OldVector::add(OldVector v) {
//	x = x + v.x;
//	y = y + v.y;
//	z = z + v.z;
//}
//
//void OldVector::sub(OldVector v) {
//	x = x - v.x;
//	y = y - v.y;
//	z = z - v.z;
//}
//
//void OldVector::mul(uint8 n) {
//	x = x * n;
//	y = y * n;
//	z = z * n;
//}
//
//void OldVector::div(uint8 n) {
//	if (n != 0) {
//		x = x / n;
//		y = y / n;
//		z = z / n;
//	}
//}
//
//// Advanced Operations /////////////////////////////////////////////////////////
//uint8 OldVector::mag() {
//	return (uint8)round(sqrt(double(x*x + y*y + z*z)));
//}
//
//void OldVector::normalise() {
//	uint8 m = mag();
//	div(m);
//}
//
//void OldVector::bound(uint8 n) {
//	// TODO: The is technically not binding the speed, which is the magnitude
//	if(x > n) x = n;
//	if(y > n) y = n;
//	if(z > n) z = n;
//}
//
//void OldVector::rand(uint8 min, uint8 max) {
//	// TODO
//}
//
//bool OldVector::empty() {
//	bool result = true;
//
//	if(x) result = false;
//	else if(y) result = false;
//	else if(z) result = false;
//
//	return result;
//}
//
//// Other ///////////////////////////////////////////////////////////////////////
//std::ostream& operator <<(std::ostream& os, const OldVector& v) {
//	os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
//    return os;
//}
//
//
//// -----------------------------------------------------------------------------
//// Constructors ////////////////////////////////////////////////////////////////
//Boid::Boid(OldVector pos, OldVector vel, uint8 id_) {
//	position = pos;
//	velocity = vel;
//	id = id_;
//	nCount = 0;
//}
//
////Boid::Boid() {
////	//TODO: Implement random functionality for initial params
////}
//
//// Getters /////////////////////////////////////////////////////////////////////
//uint8 Boid::getID () {
//	int i = 0;
//
//  return id;
//}
//
//OldVector Boid::getVelocity () {
//  return velocity;
//}
//
//OldVector Boid::getPosition() {
//	return position;
//}
//
//uint8* Boid::getNeighbours() {
//	return neighbours;
//}
//
//uint8 Boid::getNeighbour(uint8 neighbourID) {
//	return neighbours[neighbourID];
//}
//
//uint8 Boid::getNeighbourCount() {
//	return nCount;
//}
//
//// Setters /////////////////////////////////////////////////////////////////////
//void Boid::addNeighbour(uint8 neighbourID) {
//	neighbours[nCount] = neighbourID;
//	nCount++;
//}
//
//void Boid::resetNeighbours() {
//	nCount = 0;
//}
//
//void Boid::setVelocity(OldVector newVelocity) {
////	std::cout << "Boid " << id << " changed velocity from " << velocity << " to ";
//	velocity = newVelocity;
////	std::cout << velocity << std::endl;
//}
//
//void Boid::update(OldVector velocity) {
//	std::cout << "Boid " << id << " moved from " << position << " to ";
//	position.add(velocity);
//	std::cout << position << std::endl;
//}
//
//// Other ///////////////////////////////////////////////////////////////////////
//void Boid::draw() {
//	//TODO: Implement draw routine
//	//TODO: Only use in inefficient version, drawing should be done elsewhere
//}
//
//void Boid::printBoidInfo() {
//	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
//	std::cout << "Boid Velocity: " << velocity << std::endl;
//	std::cout << "Boid Position: " << position << std::endl;
//	std::cout << "===================================" << std::endl;
//}
////==============================================================================


// Functions ===================================================================
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	locationID = 16;		// TODO: Generate random initial ID
	dbg = true;				// Enable debug or not
	bool stop = false;		// Stop condition

	// Read in the command -----------------------------------------------------
	uint32 command[MAX_CMD_LEN];
	bool ignoreCmd;

//	while(!stop) {
//		// First read the command header
//		cmdHeadIn: for(int i = 0; i < CMD_HEADER_LEN; i++) {
//			command[i] = input.read();
//			//input.empty();
//		}
//
//		// If the command is not a broadcast and not addressed to me, ignore it,
//		// but still have to read the input
//		// TODO: Find a way of not having to read the input
//		if((command[0] != 0) && (command[0] != locationID)) {
//			ignoreCmd = true;
//			for(int i = 0; i < command[3]; i++) {
//				input.read();
//			}
//		} else {
//			// Else, read the command body
//			ignoreCmd = false;
//			cmdBodyIn: for(int i = 0; i < command[3]; i++) {
//				command[CMD_HEADER_LEN + i] = input.read();
//			}
//		}
//
//		printCommand(command, false);
//		// ---------------------------------------------------------------------
//
//		// Process the command -------------------------------------------------
//		if(!ignoreCmd) {
//			uint32 data[MAX_CMD_BODY_LEN];
//			uint32 len = 0;
//
//			switch(command[2]) {
//				case 0:
//					break;
//				case CMD_PING:
//					data[0] = 121;		// FPGA ID
//					len = 1;
//					createCommand(command, 1, CMD_PING_REPLY, len, data);
//
//					cmdOut: for(int i = 0; i < CMD_HEADER_LEN + command[3]; i++) {
//						output.write(command[i]);
//					}
//					printCommand(command, true);
//					break;
//				case CMD_KILL:
//					break;
//				case CMD_PING_REPLY:
//					break;
//				case CMD_INIT:
//					locationID = command[CMD_HEADER_LEN + 0];
//
//					initBoidCount = command[CMD_HEADER_LEN + 1];
//					initialiseBoids(initBoidCount);
//
//					// Store location neighbours
//					for(int i = 0; i < MAX_NEIGHBOURS; i++) {
//						locationNeighbours[i] = command[CMD_HEADER_LEN + 2 + i];
//					}
//
//					// Store location position
//					for(int i = 0; i < 8; i++) {
//						locationPosition[i] = command[CMD_HEADER_LEN + 2 +
//													  MAX_NEIGHBOURS + i];
//					}
//					break;
//				case CMD_BEGIN: {
//						// TODO: Begin the simulation
//						// While....
//						uint8 loopCounter = 1;
//						uint8 loopLimit = 3;
//						while(loopCounter <= loopLimit) {
//							std::cout << "-" << loopCounter << "-----------------"
//								<< "-----------------------------" << std::endl;
//							loopCounter++;
//						}
//						break;
//					}
//				case CMD_LOAD_INFO:
//					break;
//				case CMD_LOAD_ACT:
//					break;
//				case CMD_LOC_UPDATE:
//					break;
//				case CMD_BOID:
//					break;
//				default:
//					std::cerr << "UNKNOWN COMMAND" << std::endl;
//					break;
//			}
//		}
//		stop = true;
//		// ---------------------------------------------------------------------
//	}

//	// While....
//	uint8 loopCounter = 1;
//	uint8 loopLimit = 3;
//	while(loopCounter <= loopLimit) {
//		std::cout << "-" << loopCounter <<
//				"----------------------------------------------" << std::endl;
//		for(uint8 b = 0; b < boidCount; b++) {
//			// Calculate the boid's neighbours
//			Boid* bob = boidList[b];
//			calcNeighbours(bob);
//
//			// If the boid has any neighbours calculate the modifications to
//			// the boid's position and velocity
//			if(bob->getNeighbourCount() > 0) {
//				Vector alignMod = alignment(bob);
//				Vector groupMod = cohesion(bob);
//				Vector repelMod = separation(bob);
//
//				// Cap values and apply the modifications
//				Vector totalMod;
//				totalMod.add(alignMod);
//				totalMod.add(groupMod);
//				totalMod.add(repelMod);
//
//				if(!totalMod.empty()) {
//					//TODO: Should this be here or in each function?
//					totalMod.bound(MAXSPEED);
//
//					bob->setVelocity(totalMod);
//					bob->update(totalMod);
//					bob->draw();
//				}
//
//				bob->resetNeighbours();
//			}
//		}
//
//		loopCounter++;
//	}
//
//	for(int i = 0; i < paramData[0]; i++) {
//		boidList[i]->printBoidInfo();
//	}
}

///**
// * Create a command to send from the locations to the controller.
// *
// * If the command is a broadcast command, then no data is to be sent.
// *
// * Apparently, it is not possible to find the length of an array, given a
// * pointer to that array. Therefore, the length of the data has to be supplied.
// */
//void createCommand(uint32 *command, uint32 to, uint32 type, uint32 len, uint32 *data) {
//	command[0] = to;
//	command[1] = locationID;
//	command[2] = type;
//	command[3] = (command[0] == BROADCAST)? (uint32)0 : len;
//	command[4] = 0;
//
//	dataToCmd: for(int i = 0; i < command[3]; i++) {
//		command[5 + i] = data[i];
//	}
//}
//
///**
// * Parses the supplied command and prints it out to the terminal
// *
// * FIXME: Testbench file cannot have same method name
// */
//void printCommand(uint32 *command, bool send) {
//	if(send) {
//		if(command[0] == 0) {
//			std::cout << "-> TX, " << locationID << " sent broadcast to " << command[0] << ": ";
//		} else {
//			std::cout << "-> TX, " << locationID << " sent command to " << command[0] << ": ";
//		}
//	} else {
//		if(command[0] == 0) {
//			std::cout << "<- RX, " << locationID << " received broadcast from " << command[1] << ": ";
//		} else {
//			std::cout << "<- RX, " << locationID << " received command from " << command[1] << ": ";
//		}
//	}
//
//	switch(command[2]) {
//		case(0):
//			std::cout << "do something";
//			break;
//		case CMD_PING:
//			std::cout << "location ping";
//			break;
//		case CMD_KILL:
//			std::cout << "kill simulation";
//			break;
//		case CMD_PING_REPLY:
//			std::cout << "location ping response";
//			break;
//		case CMD_INIT:
//			std::cout << "initialise location (" << command[0] << " becomes "
//				<< command[CMD_HEADER_LEN + 0] << " with " <<
//				command[CMD_HEADER_LEN + 1] << " boids)";
//			break;
//		case CMD_BEGIN:
//			std::cout << "begin the simulation";
//			std::cout << " " << locationID;
//			break;
//		case CMD_LOAD_INFO:
//			std::cout << "location load information";
//			break;
//		case CMD_LOAD_ACT:
//			std::cout << "load-balancing decision";
//			break;
//		case CMD_LOC_UPDATE:
//			std::cout << "new location parameters";
//			break;
//		case CMD_BOID:
//			std::cout << "boid";
//			break;
//		default:
//			std::cout << "UNKNOWN COMMAND";
//			break;
//	}
//
//	std::cout << std::endl;
//
//
//	if(dbg) {
//		std::cout << "\t";
//		for(int i = 0; i < CMD_HEADER_LEN; i++) {
//			std::cout << command[i] << " ";
//		}
//
//		std::cout << "|| ";
//
//		for(int i = 0; i < command[3]; i++) {
//			std::cout << command[CMD_HEADER_LEN + i] << " ";
//		}
//		std::cout << std::endl;
//	}
//}

//void initialiseBoids(uint32 initBoidCount) {
//	// TODO: Create random initial positions and velocity
////	Vector initPos;
////	Vector initVel;
//
////	for(int i = 0; i < initBoidCount; i++) {
////		initPos.rand(0, 10);
////		initVel.rand(0, 3);
////		boidList[i] = new Boid(initPos, initVel, i + 1);
////	}
//
//	// The below is needed until a random function is created
//	//
//	// Boids can have no initial velocity as the attraction and repulsion rules
//	// will provide initial velocities. However, boids 2 and 3 did not move as
//	// their attraction and repulsion cancelled each other out. Hence, a
//	// different initial velocity was needed and now they just fly parallel.
//	OldVector initVel = OldVector(0, 0, 0);
//	OldVector altInitVel = OldVector(-2, -4, 0);
//
//	OldVector v1 = OldVector(2, 13, 0);
//	OldVector v2 = OldVector(6, 12, 0);
//	OldVector v3 = OldVector(5, 10, 0);
//
//	Boid b1 = Boid(v1, initVel, 1);
//	Boid b2 = Boid(v2, initVel, 2);
//	Boid b3 = Boid(v3, initVel, 3);
//
//	boidList[0] = &b1;
//	boidList[1] = &b2;
//	boidList[2] = &b3;
//
//	// Using the 'new' command as below, caused errors during synthesis, so
//	// the above boid creation method gets around this issue
//
////	boidList[3] = new Boid(Vector(9,8,0), initVel, 4);
////	boidList[4] = new Boid(Vector(8,7,0), initVel, 5);
////	boidList[5] = new Boid(Vector(7,5,0), initVel, 6);
////	boidList[6] = new Boid(Vector(11,6,0), initVel, 7);
////	boidList[7] = new Boid(Vector(10,5,0), initVel, 8);
////	boidList[8] = new Boid(Vector(11,4,0), initVel, 9);
////	boidList[9] = new Boid(Vector(4,3,0), initVel, 10);
//
//	std::cout << "===============================================" << std::endl;
//	std::cout << initBoidCount << " boids initialised." << std::endl;
//	std::cout << "===============================================" << std::endl;
//}
//
//void setupEnvironment(uint32 *data) {
//	// TODO: Is this needed?
//}
//
//void calcNeighbours(Boid* b) {
//	// For each boid
//	for (uint i = 0; i < boidCount; i++) {
//		// If the boid is not us
//		if(boidList[i]->getID() != b->getID()) {
//			uint8 dist = calcDistance(b->getPosition(), boidList[i]->getPosition());
//			// If the boid is within the vision radius, it is a neighbour
//			if(dist < VISIONRADIUS) {
//				b->addNeighbour(boidList[i]->getID());
//			}
//		}
//	}
//
//	// Calculate obstacles
//	// TODO: Determine suitable way of representing obstacles
//	// TODO: Overhaul neighbour list to hold both boids and obstacles
////	Vector obtacles[] = {Vector(1,-2,0), Vector(2,-2,0), Vector(3,-2,0),
////		Vector(4,-2,0), Vector(5,-2,0), Vector(6,-2,0), Vector(7,-2,0),
////		Vector(8,-2,0), Vector(9,-2,0), Vector(10,-2,0)};
////
////	for (uint i = 0; i < 9; i++) {
////		uint8 dist = calcDistance(b->getPosition(), obtacles[i]);
////		// If the obstacle is within the vision radius, it is a neighbour
////		if(dist < VISIONRADIUS) {
////			b->addNeighbour(boidList[i]->getID());
////		}
////	}
//
//	// Display neighbouring boids
////	std::cout << "Boid " << b->getID() << " has " << b->getNeighbourCount() <<
////		" neighbours: ";
////	for (int i = 0; i < b->getNeighbourCount(); i++) {
////		std::cout << b->getNeighbour(i) << ", ";
////	}
////	std::endl (std::cout);
//}
//
///**
// * Calculates the Euclidean distance between two vectors
// * TODO: Should be moved to Vector class?
// */
//uint8 calcDistance(OldVector p1, OldVector p2) {
//	double xs = pow(double(p1.x - p2.x), 2);
//	double ys = pow(double(p1.y - p2.y), 2);
//	double zs = pow(double(p1.z - p2.z), 2);
//
//	double dist = sqrt(xs + ys + zs);
//
//	return (uint8)(round(dist));
//}
//
///**
// * Alignment: iterate through a boid's neighbours, summing their velocities.
// * Then divide by the number of neighbours and normalise.
// *
// * 1) Search for neighbouring boids
// * 2) Average the velocity of these boids
// * 3) The steering vector is the difference between the average and the current
// * 		boid's velocity
// */
//OldVector alignment(Boid* b) {
//	OldVector alignment;
//	for (int i = 0; i < b->getNeighbourCount(); i++) {
//		alignment.add(boidList[(b->getNeighbour(i)) - 1]->getVelocity());
//	}
//
//	alignment.div(b->getNeighbourCount());
//	alignment.normalise();
//	return alignment;
//}
//
///**
// * Cohesion: causes the boids to steer towards one another. Sum the position of
// * all the neighbouring boids, divide by neighbour count, process and normalise.
// *
// * 1) Search for neighbouring boids
// * 2) Compute the average position of these boids
// * 3) Subtract the current boid position from the average
// */
//OldVector cohesion(Boid* b) {
//	OldVector cohesion;
//	for (int i = 0; i < b->getNeighbourCount(); i++) {
//		cohesion.add(boidList[(b->getNeighbour(i)) - 1]->getPosition());
//	}
//
//	cohesion.div(b->getNeighbourCount());
//	cohesion.sub(b->getPosition());
//	cohesion.normalise();
//	return cohesion;
//}
//
///**
// * Separation: causes the boid to steer away from its neighbours, i.e. it needs
// * to be close, but not too close. The distance between neighbouring boids is
// * added to the computation vector
// *
// * 1) Search for neighbouring boids
// * 2) For each neighbouring boid:
// * 	a) Subtract the position of the current boid and the neighbouring boid
// * 	b) Normalise
// * 	c) Apply a 1/r weighting (the position offset vector scaled by 1/r^2)
// * 3) Sum these values to produce the overall steering force
// */
//OldVector separation(Boid* b) {
//	OldVector separation;
//	OldVector dist;
//	for (int i = 0; i < b->getNeighbourCount(); i++) {
//		dist = boidList[(b->getNeighbour(i)) - 1]->getPosition();
//		dist.sub(b->getPosition());
//		separation.add(dist);
//	}
//
//	separation.div(b->getNeighbourCount());
//	separation.mul(-1);
//	separation.normalise();
//	return separation;
//}
//==============================================================================
