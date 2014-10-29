#include "location.h"

#define SIZEOF_ARRAY( a ) (sizeof( a ) / sizeof( a[ 0 ] ))

#define MAXPARAMCOUNT 20
#define MAXBOIDS 10
#define MAXNEIGHBOURS 20

#define CMD_HEADER_LEN	5	//
#define MAX_CMD_BODY_LEN 20	//
#define MAX_CMD_LEN		CMD_HEADER_LEN + MAX_CMD_BODY_LEN

#define CMD_PING		1	// Controller asking how many locations their are
#define CMD_KILL		2	// Controller stopping the simulation
#define CMD_PING_REPLY	3	// Location response to controller ping
#define CMD_INIT		4	// Controller initiation command

#define VISIONRADIUS 3
#define MAXSPEED 5

// Class Headers ===============================================================
class Vector {
	public:
		int8 x;
		int8 y;
		int8 z;

		Vector();
		Vector(int8 x_, int8 y_, int8 z_);

		void add(Vector v);
		void sub(Vector v);
		void mul(uint8 n);
		void div(uint8 n);
		uint8 mag();
		void normalise();
		void bound(uint8 n);
		bool empty();

		// TODO: Is this needed?
		friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

class Boid {
	Vector position;
	Vector velocity;
	uint8 id;
	uint8 nCount;
	uint8 neighbours[MAXNEIGHBOURS];

	public:
		Boid(Vector pos, Vector vel, uint8 id);
		uint8 getID ();
		Vector getVelocity();
		Vector getPosition();
		uint8* getNeighbours();
		uint8 getNeighbour(uint8 neighbourID);
		uint8 getNeighbourCount();

		void setVelocity(Vector newVelocity);
		void update(Vector velocity);
		void addNeighbour(uint8 neighbourID);
		void resetNeighbours();

		void draw();
		void printBoidInfo();
};
//==============================================================================


// Function Headers ============================================================
//void initialise(uint32 *data);
//
//void calcNextBoidPositions();
//
//void sendLoadInfo();
void createCommand(uint32 *command, uint32 to, uint32 type, uint32 *data);
void printCommand(uint32* command, bool send);

void setupEnvironment(uint32 *data);
void calcNeighbours(Boid* b);
uint8 calcDistance(Vector p1, Vector p2);
Vector alignment(Boid* b);
Vector cohesion(Boid* b);
Vector separation(Boid* b);

// Parameter string
Boid* boidList[MAXBOIDS];			// The indices correspond to the boid ID
Boid* neighbours[MAXNEIGHBOURS];
uint8 boidCount;

uint8 locationID;
uint8 initBoidCount;
//==============================================================================


// Classes =====================================================================
// Constructors ////////////////////////////////////////////////////////////////
Vector::Vector() {
	x = 0;
	y = 0;
	z = 0;
}

Vector::Vector(int8 x_, int8 y_, int8 z_) {
	x = x_;
	y = y_;
	z = z_;
}

// Basic Operations ////////////////////////////////////////////////////////////
void Vector::add(Vector v) {
	x = x + v.x;
	y = y + v.y;
	z = z + v.z;
}

void Vector::sub(Vector v) {
	x = x - v.x;
	y = y - v.y;
	z = z - v.z;
}

void Vector::mul(uint8 n) {
	x = x * n;
	y = y * n;
	z = z * n;
}

void Vector::div(uint8 n) {
	if (n != 0) {
		x = x / n;
		y = y / n;
		z = z / n;
	}
}

// Advanced Operations /////////////////////////////////////////////////////////
uint8 Vector::mag() {
	return (uint8)round(sqrt(double(x*x + y*y + z*z)));
}

void Vector::normalise() {
	uint8 m = mag();
	div(m);
}

void Vector::bound(uint8 n) {
	//TODO: The is technically not binding the speed, which is the magnitude
	if(x > n) x = n;
	if(y > n) y = n;
	if(z > n) z = n;
}

bool Vector::empty() {
	bool result = true;

	if(x) result = false;
	else if(y) result = false;
	else if(z) result = false;

	return result;
}

// Other ///////////////////////////////////////////////////////////////////////
std::ostream& operator <<(std::ostream& os, const Vector& v) {
	os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
    return os;
}


// -----------------------------------------------------------------------------
// Constructors ////////////////////////////////////////////////////////////////
Boid::Boid(Vector pos, Vector vel, uint8 id_) {
	position = pos;
	velocity = vel;
	id = id_;
	nCount = 0;
}

//Boid::Boid() {
//	//TODO: Implement random functionality for initial params
//}

// Getters /////////////////////////////////////////////////////////////////////
uint8 Boid::getID () {
	int i = 0;

  return id;
}

Vector Boid::getVelocity () {
  return velocity;
}

Vector Boid::getPosition() {
	return position;
}

uint8* Boid::getNeighbours() {
	return neighbours;
}

uint8 Boid::getNeighbour(uint8 neighbourID) {
	return neighbours[neighbourID];
}

uint8 Boid::getNeighbourCount() {
	return nCount;
}

// Setters /////////////////////////////////////////////////////////////////////
void Boid::addNeighbour(uint8 neighbourID) {
	neighbours[nCount] = neighbourID;
	nCount++;
}

void Boid::resetNeighbours() {
	nCount = 0;
}

void Boid::setVelocity(Vector newVelocity) {
//	std::cout << "Boid " << id << " changed velocity from " << velocity << " to ";
	velocity = newVelocity;
//	std::cout << velocity << std::endl;
}

void Boid::update(Vector velocity) {
	std::cout << "Boid " << id << " moved from " << position << " to ";
	position.add(velocity);
	std::cout << position << std::endl;
}

// Other ///////////////////////////////////////////////////////////////////////
void Boid::draw() {
	//TODO: Implement draw routine
	//TODO: Only use in inefficient version, drawing should be done elsewhere
}

void Boid::printBoidInfo() {
	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
	std::cout << "Boid Velocity: " << velocity << std::endl;
	std::cout << "Boid Position: " << position << std::endl;
	std::cout << "===================================" << std::endl;
}
//==============================================================================


// Functions ===================================================================
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	// TODO: Generate random initial ID
	locationID = 16;

	// Read in the command -----------------------------------------------------
	uint32 command[MAX_CMD_LEN];
	bool ignoreCmd;

	// First read the command header
	cmdHeadIn: for(int i = 0; i < CMD_HEADER_LEN; i++) {
		command[i] = input.read();
	}

	// If the command is not a broadcast and not addressed to me, ignore it,
	// but still have to read the input
	// TODO: Find a way of not having to read the input
	if((command[0] != 0) && (command[0] != locationID)) {
		ignoreCmd = true;
		for(int i = 0; i < command[3]; i++) {
			input.read();
		}
	} else {
		// Else, read the command body
		ignoreCmd = false;
		cmdBodyIn: for(int i = 0; i < command[3]; i++) {
			command[CMD_HEADER_LEN + i] = input.read();
		}
	}

	printCommand(command, false);
	// -------------------------------------------------------------------------

	// Process the command -----------------------------------------------------
	if(!ignoreCmd) {
		uint32 data[MAX_CMD_BODY_LEN];

		switch(command[2]) {
			case 0:
				break;
			case CMD_PING:
				data[0] = 121;		// FPGA ID
				createCommand(command, 1, CMD_PING_REPLY, data);

				cmdOut: for(int i = 0; i < CMD_HEADER_LEN + command[3]; i++) {
					output.write(command[i]);
				}
				printCommand(command, true);
				break;
			case CMD_KILL:
				break;
			case CMD_PING_REPLY:
				break;
			case CMD_INIT:
				locationID = command[CMD_HEADER_LEN + 0];
				initBoidCount = command[CMD_HEADER_LEN + 1];
				break;
			default:
				std::cerr << "UNKNOWN COMMAND" << std::endl;
				break;
		}
	}
	// -------------------------------------------------------------------------

//
//	if(command[1] != 0) {
//		std::cout << "Received command from location " << command[1] << std::endl;
//	} else if(command[0] != 0) {
//		std::cout << "Received targeted command from Controller" << std::endl;
//
//		if(command[2] == CMD_INIT) {
//			std::cout << "Location " << command[CMD_HEADER_LEN] << "("
//				<< locationID << ") initialising " <<
//				command[CMD_HEADER_LEN + 1] << " boids." << std::endl;
//
//			locationID = command[CMD_HEADER_LEN];
//			initBoidCount = command[CMD_HEADER_LEN + 1];
//		}
//
//	} else {
//		std::cout << "Received broadcast command from Controller" << std::endl;
//
//		switch(command[2]) {
//			case(CMD_PING):
//				std::cout << "Sending ping reply" << std::endl;
//
////				uint32 data[1] = 121;
////				command = composeCommand(0, locationID, CMD_PING_REPLY, data);
////				cmdOut: for(int i = 0; i < CMD_HEADER_LEN + SIZEOF_ARRAY(data); i++) {
////					output.write(command[i]);
////				}
//
//				output.write(0);				// To
//				output.write(locationID);		// From
//				output.write(CMD_PING_REPLY);	// Command type
//				output.write(1);				// Length
//				output.write(0);				// N/A
//
//				output.write(121);				// Data (FPGA ID)
//
//				break;
//			case(CMD_KILL):
//				std::cout << "Killing simulation" << std::endl;
////				killSimulation();
//				break;
//			default:
//				std::cerr << "Unknown command code: " << command[2] << std::endl;
//				break;
//		}
//	}

//	// Setup the environment
//	setupEnvironment(paramData);
//
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

void createCommand(uint32 *command, uint32 to, uint32 type, uint32 *data) {
	command[0] = to;
	command[1] = locationID;
	command[2] = type;
	command[3] = sizeof(data)/sizeof(data[0]);

	dataToCmd: for(int i = 0; i < command[3]; i++) {
		command[4 + i] = data[i];
	}
}

/**
 * Parses the supplied command and prints it out to the terminal
 */
void printCommand(uint32 *command, bool send) {
	if(send) {
		if(command[0] == 0) {
				std::cout << locationID << " sent broadcast to " << command[0] << ": ";
			} else {
				std::cout << locationID << " sent command to " << command[0] << ": ";
			}
	} else {
		if(command[0] == 0) {
			std::cout << locationID << " received broadcast from " << command[1] << ": ";
		} else {
			std::cout << locationID << "received command from " << command[1] << ": ";
		}
	}

	switch(command[2]) {
		case(0):
			std::cout << "do something";
			break;
		case CMD_PING:
			std::cout << "location ping";
			break;
		case CMD_KILL:
			std::cout << "kill simulation";
			break;
		case CMD_PING_REPLY:
			std::cout << "location ping response";
			break;
		case CMD_INIT:
			std::cout << "initialise location";
			break;
		default:
			std::cout << "UNKNOWN COMMAND";
			break;
	}

	std::cout << std::endl;
}

void setupEnvironment(uint32 *data) {
	// Number of boids
	//TODO: Adjust maximum of boid list depending on number of boids specified
	// though I don't think this can be done...

	//TODO: Create random initial positions and velocity
//	for(int i = 0; i < data[0]; i++) {
//		uint8 pos[DIMENSIONS] = {1,2,3};
//		boidList[i] = new Boid(pos, 2, i+1);
//		boidList[i]->printBoidInfo();
//	}

//	boidCount = MAXBOIDS;
	boidCount = 3;

	// Boids can have no initial velocity as the attraction and repulsion rules
	// will provide initial velocities
	Vector initVel = Vector(0, 0, 0);
	Vector altInitVel = Vector(-2, -4, 0);
	// Boids 2 and 3 did not move as their attraction and repulsion cancelled
	// each other out. Hence, a different initially velocity was needed and now
	// they just fly parallel to each other.

	boidList[0] = new Boid(Vector(2,13,0), initVel, 1);
	boidList[1] = new Boid(Vector(6,12,0), initVel, 2);
	boidList[2] = new Boid(Vector(5,10,0), altInitVel, 3);
//	boidList[3] = new Boid(Vector(9,8,0), initVel, 4);
//	boidList[4] = new Boid(Vector(8,7,0), initVel, 5);
//	boidList[5] = new Boid(Vector(7,5,0), initVel, 6);
//	boidList[6] = new Boid(Vector(11,6,0), initVel, 7);
//	boidList[7] = new Boid(Vector(10,5,0), initVel, 8);
//	boidList[8] = new Boid(Vector(11,4,0), initVel, 9);
//	boidList[9] = new Boid(Vector(4,3,0), initVel, 10);



	std::cout << "===============================================" << std::endl;
	std::cout << data[0] << " boids initialised in grid of size " << data[1] <<
		" by " << data[2] << std::endl;
	std::cout << "===============================================" << std::endl;
}

void calcNeighbours(Boid* b) {
	// For each boid
	for (uint i = 0; i < boidCount; i++) {
		// If the boid is not us
		if(boidList[i]->getID() != b->getID()) {
			uint8 dist = calcDistance(b->getPosition(), boidList[i]->getPosition());
			// If the boid is within the vision radius, it is a neighbour
			if(dist < VISIONRADIUS) {
				b->addNeighbour(boidList[i]->getID());
			}
		}
	}

	// Calculate obstacles
	// TODO: Determine suitable way of representing obstacles
	// TODO: Overhaul neighbour list to hold both boids and obstacles
//	Vector obtacles[] = {Vector(1,-2,0), Vector(2,-2,0), Vector(3,-2,0),
//		Vector(4,-2,0), Vector(5,-2,0), Vector(6,-2,0), Vector(7,-2,0),
//		Vector(8,-2,0), Vector(9,-2,0), Vector(10,-2,0)};
//
//	for (uint i = 0; i < 9; i++) {
//		uint8 dist = calcDistance(b->getPosition(), obtacles[i]);
//		// If the obstacle is within the vision radius, it is a neighbour
//		if(dist < VISIONRADIUS) {
//			b->addNeighbour(boidList[i]->getID());
//		}
//	}

	// Display neighbouring boids
//	std::cout << "Boid " << b->getID() << " has " << b->getNeighbourCount() <<
//		" neighbours: ";
//	for (int i = 0; i < b->getNeighbourCount(); i++) {
//		std::cout << b->getNeighbour(i) << ", ";
//	}
//	std::endl (std::cout);
}

/**
 * Calculates the Euclidean distance between two vectors
 * TODO: Should be moved to Vector class?
 */
uint8 calcDistance(Vector p1, Vector p2) {
	double xs = pow(double(p1.x - p2.x), 2);
	double ys = pow(double(p1.y - p2.y), 2);
	double zs = pow(double(p1.z - p2.z), 2);

	double dist = sqrt(xs + ys + zs);

	return (uint8)(round(dist));
}

/**
 * Alignment: iterate through a boid's neighbours, summing their velocities.
 * Then divide by the number of neighbours and normalise.
 *
 * 1) Search for neighbouring boids
 * 2) Average the velocity of these boids
 * 3) The steering vector is the difference between the average and the current
 * 		boid's velocity
 */
Vector alignment(Boid* b) {
	Vector alignment;
	for (int i = 0; i < b->getNeighbourCount(); i++) {
		alignment.add(boidList[(b->getNeighbour(i)) - 1]->getVelocity());
	}

	alignment.div(b->getNeighbourCount());
	alignment.normalise();
	return alignment;
}

/**
 * Cohesion: causes the boids to steer towards one another. Sum the position of
 * all the neighbouring boids, divide by neighbour count, process and normalise.
 *
 * 1) Search for neighbouring boids
 * 2) Compute the average position of these boids
 * 3) Subtract the current boid position from the average
 */
Vector cohesion(Boid* b) {
	Vector cohesion;
	for (int i = 0; i < b->getNeighbourCount(); i++) {
		cohesion.add(boidList[(b->getNeighbour(i)) - 1]->getPosition());
	}

	cohesion.div(b->getNeighbourCount());
	cohesion.sub(b->getPosition());
	cohesion.normalise();
	return cohesion;
}

/**
 * Separation: causes the boid to steer away from its neighbours, i.e. it needs
 * to be close, but not too close. The distance between neighbouring boids is
 * added to the computation vector
 *
 * 1) Search for neighbouring boids
 * 2) For each neighbouring boid:
 * 	a) Subtract the position of the current boid and the neighbouring boid
 * 	b) Normalise
 * 	c) Apply a 1/r weighting (the position offset vector scaled by 1/r^2)
 * 3) Sum these values to produce the overall steering force
 */
Vector separation(Boid* b) {
	Vector separation;
	Vector dist;
	for (int i = 0; i < b->getNeighbourCount(); i++) {
		dist = boidList[(b->getNeighbour(i)) - 1]->getPosition();
		dist.sub(b->getPosition());
		separation.add(dist);
	}

	separation.div(b->getNeighbourCount());
	separation.mul(-1);
	separation.normalise();
	return separation;
}
//==============================================================================
