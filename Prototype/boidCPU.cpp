#include "boidCPU.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>		/* cout */
#include <math.h>       /* sqrt */

#define MAX_BOIDS				30
#define MAX_VELOCITY			10
#define MAX_FORCE				1

#define AREA_WIDTH				720
#define AREA_HEIGHT				720

#define VISION_RADIUS			100
#define MAX_BOIDCPU_NEIGHBOURS	9
#define MAX_NEIGHBOURING_BOIDS	90		// TODO: Decide on appropriate value?

// Function headers
static void initialisation (void);
static void identify (void);
static void simulationSetup (void);
static void findNeighbours (void);
static void calcNextBoidPositions (void);
static void loadBalance (void);
static void moveBoids (void);
static void updateDisplay (void);

// Define the states
enum States {INIT, IDEN, SIMS, NBRS, BOID, LOAD, MOVE, DRAW, MAX_STATES};
States state;

// Define a transition type
typedef struct {
	States state;
	void (*function)(void);
} Transition;

// Create the transition table
Transition trans[MAX_STATES] = {
	{INIT, &initialisation},
	{IDEN, &identify},
	{SIMS, &simulationSetup},
	{NBRS, &findNeighbours},
	{BOID, &calcNextBoidPositions},
	{LOAD, &loadBalance},
	{MOVE, &moveBoids},
	{DRAW, &updateDisplay}
};
#define TRANS_COUNT (sizeof(trans)/sizeof(*trans))

// Define variables
// FIXME: Should these really be global?
int8 boidCPUID;
int8 boidCount;
Boid boids[MAX_BOIDS];
uint8 neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS];

void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	state = INIT;
	// Can a while(1) be used in h/w is there one implicitly elsewhere?
	while (state != DRAW) {
//		for(int i = 0; i < TRANS_COUNT; i++) {
//			if (state == trans[i].state) {
//				(trans[i].function)();
//			}
//		}
		// Catch invalid values
		(trans[state].function)();
	}
}

// State methods
void initialisation() {
	std::cout << "-Initialising BoidCPU..." << std::endl;

	boidCPUID = 6;
	int fpgaID = 123;

	state = IDEN;
}

void identify() {
	std::cout << "-Waiting for ping from Boid Controller..." << std::endl;

	std::cout << "-Responded to ping" << std::endl;

	state = SIMS;
}

void simulationSetup() {
	std::cout << "-Preparing BoidCPU for simulation..." << std::endl;

	// These should be supplied by the controller over the communications link
	boidCPUID = 6;
	int12 boidCPUCoords[4] = {480, 240, 720, 480};
	int8 initialBoidCount = 10;
	// TODO: Supply the neighbouring BoidCPUs

	boidCount = initialBoidCount;

	// Use this for testing
	int testBoidCount = 10;
	Vector knownSetup[10][2] = {{Vector(695, 252, 0), Vector(-5, -9, 0)},
			{Vector(594, 404, 0), Vector(-10, -1, 0)},
			{Vector(550, 350, 0), Vector(-10, -3, 0)},
			{Vector(661, 446, 0), Vector(-6, -4, 0)},
			{Vector(539, 283, 0), Vector(-8, -2, 0)},
			{Vector(551, 256, 0), Vector(-5, 7, 0)},
			{Vector(644, 342, 0), Vector(-1, -7, 0)},
			{Vector(592, 399, 0), Vector(-9, 6, 0)},
			{Vector(644, 252, 0), Vector(-5, -8, 0)},
			{Vector(687, 478, 0), Vector(-9, 9, 0)}};

	for(int i = 0; i < testBoidCount; i++) {
		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;
		Boid boid = Boid(boidID, knownSetup[i][0], knownSetup[i][1]);
		boids[i] = boid;
	}

//	for(int8 i = 0; i < boidCount; i++) {
//		Vector position;
//		Vector velocity;
//
//		position.rand(boidCPUCoords[0], boidCPUCoords[1], boidCPUCoords[2], boidCPUCoords[3]);
//		velocity.rand(-MAX_VELOCITY, -MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY);
//
//		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;
//
//		Boid boid = Boid(boidID, position, velocity);
//		boids[i] = boid;
//	}

	state = NBRS;
}

void findNeighbours() {
	std::cout << "Finding neighbouring boids..." << std::endl;

	// The neighbouring BoidCPUs would be supplied by the controller on intialisation
	neighbouringBoidCPUs[MAX_BOIDCPU_NEIGHBOURS] = {2, 3, 1, 4, 7, 9, 8, 5};

	// Generate a list of possible neighbouring boids
	Boid possibleNeighbouringBoids[MAX_NEIGHBOURING_BOIDS];
	uint8 possibleNeighbourCount = 0;

	for (int i = 0; i < boidCount; i++) {
		possibleNeighbouringBoids[i] = boids[i];
		possibleNeighbourCount++;
	}

	for (int i = boidCount; i < MAX_BOIDCPU_NEIGHBOURS; i++) {
		if (neighbouringBoidCPUs[i] != 0) {
			// TODO: Get boids from neighbouring BoidCPUs
			// possibleNeighbouringBoids[] =
			// possibleNeighbourCount++;
		}
	}

	// Calculate a the neighbours for each boid
	for (int i = 0; i < boidCount; i++) {
		boids[i].calculateNeighbours(possibleNeighbouringBoids, possibleNeighbourCount);
	}

	// A list of the known neighbours for the boids at time step 1 (from Python)
	// Sat 10th Jan: HLS is correct
	int knownSetupNeighbours [10][5] = {{59},
			 {53, 54, 57, 58},
			 {52, 55, 56, 57, 58},
			 {52, 58, 60},
			 {53, 56},
			 {53, 55, 59},
			 {52, 53, 58, 59},
			 {52, 53, 54, 57},
			 {51, 56, 57},
			 {54}};

	state = BOID;
}

void calcNextBoidPositions() {
	std::cout << "Calculating next boid positions..." << std::endl;

	for (int i = 0; i < boidCount; i++) {
		boids[i].update();
	}

	// The next positions of the boids, rounded to ints from floats, from Python
	Vector knownSetupNextPos[10][2] = {{Vector(691, 244, 0), Vector(-4, -8, 0)},
			{Vector(586, 403, 0), Vector(-8, -2, 0)},
			{Vector(541, 349, 0), Vector(-9, -2, 0)},
			{Vector(655, 444, 0), Vector(-6, -2, 0)},
			{Vector(532, 283, 0), Vector(-7, -2, 0)},
			{Vector(547, 261, 0), Vector(-4, 5, 0)},
			{Vector(642, 337, 0), Vector(-2, -5, 0)},
			{Vector(584, 403, 0), Vector(-8, 4, 0)},
			{Vector(639, 245, 0), Vector(-5, -7, 0)},
			{Vector(680, 485, 0), Vector(-7, 7, 0)}};

	for (int i = 0; i < boidCount; i++) {
		if (!Vector::equal(boids[i].getPosition(), knownSetupNextPos[i][0])) {
			std::cout << "Boid #" << boids[i].getID() << " position differs" << std::endl;
			std::cout << "   " << boids[i].getPosition() << " vs " << knownSetupNextPos[i][0] << std::endl;
		} else {
			std::cout << "Boid #" << boids[i].getID() << " position same" << std::endl;
		}
	}

	state = LOAD;
}

void loadBalance() {
	std::cout << "Load balancing..." << std::endl;
	state = MOVE;
}

void moveBoids() {
	std::cout << "Moving boids..." << std::endl;

	for (int i = 0; i < boidCount; i++) {
		if((neighbouringBoidCPUs[0] != 0) && (boids[i].getPosition().x < neighbouringBoidCPUs[])) {

		}
	}

	state = DRAW;
}

void updateDisplay() {
	std::cout << "Updating display" << std::endl;
	//state = NBRS;
}

// Generic methods
void transmit(int to, int data) {
//	output.write('a');
}

void receive() {
//	input.read();

	// Identify the input and if it is a boid, send data to function
	acceptBoid(inputData);
}

void acceptBoid(uint32 *boidData) {
	// TODO: Parse the input data to create a boid object
	int boidID = 12;
	Vector boidPosition = Vector(12, 100, 0);
	Vector boidVelocity = Vector(10, -2, 0);
	Boid b = Boid(boidID, boidPosition, boidVelocity);

	boids[boidCount] = b;
	boidCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Classes /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//==============================================================================
// Boid ========================================================================
//==============================================================================

Boid::Boid() {
	id = 0;

	position = Vector(0, 0, 0);
	velocity = Vector(0, 0, 0);

	neighbouringBoidsCount = 0;
}

Boid::Boid(int _boidID, Vector initPosition, Vector initVelocity) {
	id = _boidID;

	position = initPosition;
	velocity = initVelocity;

	neighbouringBoidsCount = 0;

	std::cout << "Created boid #" << id << std::endl;
	printBoidInfo();
}

// TODO: Will need to make a copy of the neighbours rather than a reference so
//	that as the neighbours are updated, the neighbour list doesn't change
void Boid::calculateNeighbours(Boid *possibleNeighbours, int possibleNeighbourCount) {
	std::cout << "Calculating neighbours for boid #" << id << std::endl;

	for (int i = 0; i < possibleNeighbourCount; i++) {
		if (possibleNeighbours[i].getID() != id) {
			double distance = Vector::distanceBetween(position, possibleNeighbours[i].getPosition());
			if (distance < VISION_RADIUS) {
				neighbouringBoids[neighbouringBoidsCount] = &possibleNeighbours[i];
				neighbouringBoidsCount++;
			}
		}
	}

	std::cout << "Boid #" << id << " has " << neighbouringBoidsCount << " neighbours: ";
	for (int i = 0; i < neighbouringBoidsCount; i++) {
		std::cout << neighbouringBoids[i]->getID() << ", ";
	} std::cout << std::endl;
}

void Boid::update(void) {
	std::cout << "Updating boid #" << id << std::endl;

	if(neighbouringBoidsCount > 0) {
		acceleration.add(separate());
		acceleration.add(align());
		acceleration.add(cohesion());
	}

	velocity.add(acceleration);
	velocity.limit(MAX_VELOCITY);
	position.add(velocity);
	acceleration.mul(0);

	contain();
}

Vector Boid::align(void) {
	Vector total;

	for (int i = 0; i < neighbouringBoidsCount; i++) {
		// FIXME: This doesn't work as Boid can't have a list of Boids
		total.add(neighbouringBoids[i]->getVelocity());
		//total.add(getDummyVector());
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::separate(void) {
	Vector total;
	Vector diff;

	for (int i = 0; i < neighbouringBoidsCount; i++) {
		// FIXME: This doesn't work as Boid can't have a list of Boids
		diff = Vector::sub(position, neighbouringBoids[i]->getPosition());
		//diff = Vector::sub(position, getDummyVector());
		diff.normalise();
		total.add(diff);
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::cohesion(void) {
	Vector total;
	Vector steer;

	for (int i = 0; i < neighbouringBoidsCount; i++) {
		// FIXME: This doesn't work as Boid can't have a list of Boids
		total.add(neighbouringBoids[i]->getPosition());
		//total.add(getDummyVector());
	}

	total.div(neighbouringBoidsCount);
	steer = seek(total);
	return steer;
}

Vector Boid::seek(Vector target) {
	Vector desired = Vector::sub(target, position);
	desired.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(desired, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

void Boid::contain() {
	if(position.x > AREA_WIDTH) {
		position.x = 0;
	} else if(position.x < 0) {
		position.x = AREA_WIDTH;
	}

	if(position.y > AREA_HEIGHT) {
		position.y = 0;
	} else if(position.y < 0) {
		position.y = AREA_HEIGHT;
	}
}

Vector Boid::getVelocity(void) {
	return velocity;
}

Vector Boid::getPosition(void) {
	return position;
}

uint8 Boid::getID(void) {
	return id;
}

//uint8 Boid::getNeighbourCount(void) {
//	return neighbouringBoidsCount;
//}
//
//Boid* Boid::getNeighbours(void) {
//	return &neighbouringBoids;
//}

void Boid::printBoidInfo() {
	std::cout << "==========Info for Boid " << id << "==========" << std::endl;
	std::cout << "Boid Velocity: " << velocity << std::endl;
	std::cout << "Boid Position: " << position << std::endl;
	std::cout << "===================================" << std::endl;
}

//==============================================================================
// Vector ======================================================================
//==============================================================================

// Constructors ////////////////////////////////////////////////////////////////
Vector::Vector() {
	x = 0;
	y = 0;
	z = 0;
}

Vector::Vector(int12 x_, int12 y_, int12 z_) {
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

// Static Operations /////////////////////////////////////////////////////////
Vector Vector::add(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
	return v3;
}

Vector Vector::sub(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	return v3;
}

// FIXME: The pow function returns a double and the sqrt takes a double, these
// 	will probably not be allowed in hardware - or will be expensive.
double Vector::distanceBetween(Vector v1, Vector v2) {
	return sqrt(pow((v1.x - v2.x), 2) + pow((v1.y - v2.y), 2) + pow((v1.z - v2.z), 2));
}

bool Vector::equal(Vector v1, Vector v2) {
	if ((v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z)) {
		return true;
	} else {
		return false;
	}
}

// Advanced Operations /////////////////////////////////////////////////////////
uint8 Vector::mag() {
	return (uint8)round(sqrt(double(x*x + y*y + z*z)));
}

void Vector::setMag(uint8 mag) {
	normalise();
	mul(mag);
}

void Vector::normalise() {
	uint8 m = mag();
	div(m);
}

void Vector::limit(uint8 max) {
	if(mag() > max) {
		normalise();
		mul(max);
	}
}

void Vector::bound(uint8 n) {
	// TODO: The is technically not binding the speed, which is the magnitude
	if(x > n) x = n;
	if(y > n) y = n;
	if(z > n) z = n;
}

// http://stackoverflow.com/a/5009006
void Vector::rand(int12 xMin, int12 yMin, int12 xMax, int12 yMax) {
	x = xMin + (std::rand() % (int)(xMax - xMin + 1));
	y = yMin + (std::rand() % (int)(yMax - yMin + 1));
	z = 0;
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
