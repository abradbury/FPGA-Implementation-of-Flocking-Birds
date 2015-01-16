#ifndef __TOPLEVEL_H_
#define __TOPLEVEL_H_

#include <stdio.h>
#include <stdlib.h>
#include <ap_int.h>			// For arbitrary precision types
#include <ap_fixed.h>		// For fixed point data types
#include <hls_stream.h>

#define MAX_NEIGHBOURS 	8	// The maximum number of neighbouring locations

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;

typedef ap_uint<16> uint16;
typedef ap_int<16> int16;

typedef ap_uint<8> uint8;
typedef ap_int<8> int8;

typedef ap_int<12> int12;	// Used to represent position and negative velocity
typedef ap_int<12> uint12;

//Prototypes
void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output);

// Classes
class Vector {
	public:
		int12 x;
		int12 y;
		int12 z;

		Vector();
		Vector(int12 x_, int12 y_, int12 z_);

		void add(Vector v);
		void sub(Vector v);
		void mul(uint8 n);
		void div(uint8 n);

		uint8 mag();
		void normalise();
		void bound(uint8 n);
		bool empty();

		void setMag(uint8 mag);
		void limit(uint8 max);

		static Vector add(Vector v1, Vector v2);
		static Vector sub(Vector v1, Vector v2);
		static double distanceBetween(Vector v1, Vector v2);
		static bool equal(Vector v1, Vector v2);

		// TODO: Is this needed?
//		friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

class Boid {
	public:
		Vector position;                    // The current pixel position of the boid
		Vector velocity;                   	// The current velocity of the boid
		uint8 id;

		Boid();
		Boid(int _boidID, Vector initPosition, Vector initVelocity); // Constructor - initialise the boid

		void calculateNeighbours(Boid *possibleNeighbours, int possibleNeighbourCount);  // Calculate the neighbours for the boid
		void update(); 						// Calculate the new position of the boid
		void draw();						// Draw the boid

//		Vector getVelocity();
//		Vector getPosition();
//		uint8 getID();
//		uint8 getNeighbourCount();
//		Boid* getNeighbours();

		void printBoidInfo();

	private:
		Vector acceleration;
		uint8 neighbouringBoidsCount;
//		Boid *neighbouringBoids[MAX_NEIGHBOURS];

		Vector align();						// Calculate the alignment force
		Vector separate();					// Calculate the separation force
		Vector cohesion();					// Calculate the cohesion force

		Vector seek(Vector);				// Seek a new position
		void contain();						// Contain the boids within the simulation area
};

#endif
