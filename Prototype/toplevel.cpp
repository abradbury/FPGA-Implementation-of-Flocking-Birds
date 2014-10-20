#include "toplevel.h"

#define MAXPARAMCOUNT 20
#define MAXBOIDS 10
#define MAXNEIGHBOURS 20

#define VISIONRADIUS 3
#define MAXSPEED 5

// Class
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

		// FIXME: Not working
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

		void updateVelocity(Vector newVelocity);
		void updatePosition(Vector newPosition);
		void addNeighbour(uint8 neighbourID);
		void resetNeighbours();

		void printBoidInfo();
};

//Prototypes
void setupEnvironment(uint32 *data);
void calcNeighbours(Boid* b);
uint8 calcDistance(Boid* b1, Boid* b2);
Vector alignment(Boid* b);
Vector cohesion(Boid* b);
Vector separation(Boid* b);

// Parameter string
Boid* boidList[MAXBOIDS];			// The indices correspond to the boid ID
Boid* neighbours[MAXNEIGHBOURS];

//==============================================================================
//==============================================================================
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

//FIXME: Not currently working
std::ostream& operator<<(std::ostream& os, const Vector& v)
{
    os << v.x << ', ' << v.y << ', ' << v.z;
    return os;
}


//==============================================================================
// Constructor /////////////////////////////////////////////////////////////////
Boid::Boid(Vector pos, Vector vel, uint8 id_) {
	position = pos;
	velocity = vel;
	id = id_;
	nCount = 0;
}

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

void Boid::updateVelocity(Vector newVelocity) {
//	std::cout << "Boid " << id << " changed velocity from [" << velocity.x <<
//		", " << velocity.y << ", " << velocity.z << "] to [";

	velocity = newVelocity;

//	std::cout << velocity.x << ", " << velocity.y << ", " << velocity.z <<
//		"]" << std::endl;
}

void Boid::updatePosition(Vector newPosition) {
	std::cout << "Boid " << id << " moved from [" << position.x << ", " <<
		position.y << ", " << position.z << "] to [";

	position.add(newPosition);

	std::cout << position.x << ", " << position.y << ", " << position.z <<
		"]" << std::endl;
}

// Other ///////////////////////////////////////////////////////////////////////
void Boid::printBoidInfo() {
	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
	std::cout << "Boid Velocity: [" << velocity.x << ", " << velocity.y <<
			", " << velocity.z << "]" << std::endl;
	std::cout << "Boid Position: [" << position.x << ", " << position.y <<
			", " << position.z << "]" << std::endl;
	std::cout << "===================================" << std::endl;
}
//==============================================================================
//==============================================================================


//Top-level function
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	// Read in parameter string
	uint8 paramCount = input.read();
	uint32 paramData[MAXPARAMCOUNT];

	paramloop: for(int i = 0; i < paramCount; i++) {
		paramData[i] = input.read();
	}

	// Setup the environment
	setupEnvironment(paramData);

	// While....
	uint8 loopCounter = 1;
	uint8 loopLimit = 10;
	while(loopCounter <= loopLimit) {
		std::cout << "-" << loopCounter <<
				"----------------------------------------------" << std::endl;
		for(uint8 b = 0; b < MAXBOIDS; b++) {
			// Calculate the boid's neighbours
			Boid* bob = boidList[b];
			calcNeighbours(bob);

			// If the boid has any neighbours calculate the modifications to
			// the boid's position and velocity
			if(bob->getNeighbourCount() > 0) {
				Vector alignMod = alignment(bob);
				Vector groupMod = cohesion(bob);
				Vector repelMod = separation(bob);

				// Cap values and apply the modifications
				Vector totalMod;
				totalMod.add(alignMod);
				totalMod.add(groupMod);
				totalMod.add(repelMod);

				if(!totalMod.empty()) {
					//TODO: Should this be here or in each function?
					totalMod.bound(MAXSPEED);

					bob->updateVelocity(totalMod);
					bob->updatePosition(totalMod);
				}

				bob->resetNeighbours();
			}
		}

		loopCounter++;
	}

	for(int i = 0; i < paramData[0]; i++) {
		boidList[i]->printBoidInfo();
	}
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

	// Boids can have no initial velocity as the attraction and repulsion rules
	// will provide initial velocities
	Vector initVel = Vector(0, 0, 0);

	boidList[0] = new Boid(Vector(2,13,0), initVel, 1);
	boidList[1] = new Boid(Vector(6,12,0), initVel, 2);
	boidList[2] = new Boid(Vector(5,10,0), initVel, 3);
	boidList[3] = new Boid(Vector(9,8,0), initVel, 4);
	boidList[4] = new Boid(Vector(8,7,0), initVel, 5);
	boidList[5] = new Boid(Vector(7,5,0), initVel, 6);
	boidList[6] = new Boid(Vector(11,6,0), initVel, 7);
	boidList[7] = new Boid(Vector(10,5,0), initVel, 8);
	boidList[8] = new Boid(Vector(11,4,0), initVel, 9);
	boidList[9] = new Boid(Vector(4,3,0), initVel, 10);

	std::cout << "===============================================" << std::endl;
	std::cout << data[0] << " boids initialised in grid of size " << data[1] <<
		" by " << data[2] << std::endl;
	std::cout << "===============================================" << std::endl;
}

void calcNeighbours(Boid* b) {
	// For each boid
	for (uint i = 0; i < MAXBOIDS; i++) {
		// If the boid is not us
		if(boidList[i]->getID() != b->getID()) {
			uint8 dist = calcDistance(b, boidList[i]);
			// If the boid is within the vision radius, it is a neighbour
			if(dist < VISIONRADIUS) {
				b->addNeighbour(boidList[i]->getID());
			}
		}
	}

	// Display neighbouring boids
	std::cout << "Boid " << b->getID() << " has " << b->getNeighbourCount() <<
		" neighbours: ";
	for (int i = 0; i < b->getNeighbourCount(); i++) {
		std::cout << b->getNeighbour(i) << ", ";
	}
	std::endl (std::cout);
}

/**
 * Calculates the Euclidean distance between two boids
 */
uint8 calcDistance(Boid* b1, Boid* b2) {
	double xs = pow(double(b1->getPosition().x - b2->getPosition().x), 2);
	double ys = pow(double(b1->getPosition().y - b2->getPosition().y), 2);
	double zs = pow(double(b1->getPosition().z - b2->getPosition().z), 2);

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

