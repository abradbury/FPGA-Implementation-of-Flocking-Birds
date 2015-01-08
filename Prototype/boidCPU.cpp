#include "boidCPU.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define MAX_BOIDS		30
#define MAX_VELOCITY	10
#define MAX_FORCE		1

#define AREA_WIDTH		720
#define AREA_HEIGHT		720

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

void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS INTERFACE ap_fifo port=input
#pragma HLS INTERFACE ap_fifo port=output
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	state = INIT;
	// Can a while(1) be used in h/w is there one implicitly elsewhere?
	while (1) {
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

	int boidCPUID = 6;
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

	int8 boidCPUID = 6;
	int8 boidCPUCoords[4] = {0, 0, 240, 240};
	int8 initialBoidCount = 10;
	int8 boidCount = initialBoidCount;
	Boid boids[MAX_BOIDS];

	for(int8 i = 0; i < boidCount; i++) {
		Vector position;
		Vector velocity;

		position.rand2D(boidCPUCoords[0], boidCPUCoords[1], boidCPUCoords[2], boidCPUCoords[3]);
		velocity.rand2D(-MAX_VELOCITY, MAX_VELOCITY, -MAX_VELOCITY, MAX_VELOCITY);

		uint8 boidID = ((boidCPUID - 1) * boidCount) + i + 1;

		Boid boid = Boid(boidID, position, velocity);
		boids[boidCount] = boid;
	}

	state = NBRS;
}

void findNeighbours() {
	std::cout << "Finding neighbouring boids..." << std::endl;

	state = BOID;
}

void calcNextBoidPositions() {
	std::cout << "Calculating next boid positions..." << std::endl;

	state = LOAD;
}

void loadBalance() {
	std::cout << "Load balancing..." << std::endl;
	state = MOVE;
}

void moveBoids() {
	std::cout << "Moving boids..." << std::endl;
	state = DRAW;
}

void updateDisplay() {
	std::cout << "Updating display" << std::endl;
	state = NBRS;
}

// Generic methods
void transmit(int to, int data) {
//	output.write('a');
}

void receive() {
//	input.read();
}

////////////////////////////////////////////////////////////////////////////////
// Classes /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

// Static Operations /////////////////////////////////////////////////////////
static Vector Vector::add(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x + v2.x, v1.y + v2.y);
	return v3;
}

static Vector Vector::sub(Vector v1, Vector v2) {
	Vector v3 = Vector(v1.x - v2.x, v1.y - v2.y);
	return v3;
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
	if(1 > max) {
		//TODO
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

void Vector::rand(uint8 min, uint8 max) {
	// TODO
}

// http://stackoverflow.com/a/5009006
void Vector::rand2D(uint8 xMin, uint8 xMax, uint8 yMin, uint8 yMax) {
//		x = xMin + (rand(1) % (int)(xMax - xMin + 1));
//		y = yMin + (rand(1) % (int)(yMax - yMin + 1));
	x = 10;
	y = 10;
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

Vector::Vector(int8 x_, int8 y_, int8 z_) {
	x = x_;
	y = y_;
	z = z_;
}

Boid::Boid(int _boidID, Vector initPosition, Vector initVelocity) {

	boidID = _boidID;

	position = initPosition;
	velocity = initVelocity;

	neighbouringBoids = {0};
	neighbouringBoidsCount = 0;
}

void Boid::Update(void) {
	if(neighbouringBoidsCount > 0) {
		acceleration.add(Separate());
		acceleration.add(Align());
		acceleration.add(Cohesion());
	}

	velocity.add(acceleration);
	velocity.limit(MAX_VELOCITY);
	position.add(velocity);
	acceleration.mul(0);

	Contain();
}

Vector Boid::Align(void) {
	Vector total;

	for (Boid b : neighbouringBoids) {
		total += b.getVelocity();
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::Separate(void) {
	Vector total;
	Vector diff;

	for (Boid b : neighbouringBoids) {
		diff = Vector::sub(position, b.getPosition());
		diff.normalise();
		total.add(diff);
	}

	total.div(neighbouringBoidsCount);
	total.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(total, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

Vector Boid::Cohesion(void) {
	Vector total;

	for(Boid b : neighbouringBoids) {
		total.add(b.getPosition());
	}

	total.div(neighbouringBoidsCount);
	Vector steer = Seek(total);
	return steer;
}

Vector Boid::Seek(Vector target) {
	Vector desired = Vector::sub(target, position);
	desired.setMag(MAX_VELOCITY);

	Vector steer = Vector::sub(desired, velocity);
	steer.limit(MAX_FORCE);

	return steer;
}

void Boid::Contain() {
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
