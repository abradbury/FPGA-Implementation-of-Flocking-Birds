#ifndef __TOPLEVEL_H_
#define __TOPLEVEL_H_

#include <stdio.h>
#include <stdlib.h>
#include <ap_int.h>
#include <hls_stream.h>

#define MAX_NEIGHBOURS 	8	// The maximum number of neighbouring locations

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;

typedef ap_uint<8> uint8;
typedef ap_int<8> int8;

//Prototypes
void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output);

// Classes
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
		void rand(uint8 min, uint8 max);
		void rand2D(uint8 xMin, uint8 xMax, uint8 yMin, uint8 yMax);
		bool empty();

		void setMag(uint8 mag);
		void limit(uint8 max);

		static Vector add(Vector v1, Vector v2);
		static Vector sub(Vector v1, Vector v2);
		static double distanceBetween(Vector v1, Vector v2);

		// TODO: Is this needed?
		friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

class Boid {
	public:
		Boid();
		Boid(int _boidID, Vector initPosition, Vector initVelocity); // Constructor - initialise the boid

		void CalculateNeighbours(Boid *possibleNeighbours, int possibleNeighbourCount);  // Calculate the neighbours for the boid
		void Update(); 						// Calculate the new position of the boid
		void Draw();						// Draw the boid

		Vector getDummyVector();
		Vector getVelocity();
		Vector getPosition();
		int getID();

	private:
		Vector position;                    // The current pixel position of the boid
		Vector velocity;                   	// The current velocity of the boid
		Vector acceleration;

		int boidID;
		int neighbouringBoidsCount;

		Boid *neighbouringBoids[MAX_NEIGHBOURS];

		Vector Align();						// Calculate the alignment force
		Vector Separate();					// Calculate the separation force
		Vector Cohesion();					// Calculate the cohesion force

		Vector Seek(Vector);				// Seek a new position
		void Contain();						// Contain the boids within the simulation area
};

#endif
