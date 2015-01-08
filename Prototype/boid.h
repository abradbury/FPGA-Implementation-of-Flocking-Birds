//#ifndef __TOPLEVEL_H_
//#define __TOPLEVEL_H_
//
#include <stdio.h>
#include <stdlib.h>
#include <ap_int.h>
#include <hls_stream.h>

#include "vector.h"

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;

typedef ap_uint<8> uint8;
typedef ap_int<8> int8;
//
////Prototypes
//void toplevelthree(hls::stream<uint32> &input, hls::stream<uint32> &output);
//
//#endif


class Boid {
  public:
    Boid();                         	// Constructor - initialise the boid
    void CalculateNeighbours(int possibleNeighbours);  // Calculate the neighbours for the boid
    void Update(); 						// Calculate the new position of the boid
    void Draw();						// Draw the boid

  private:
    Vector position;                    	// The current pixel position of the boid
    Vector velocity;                   	// The current velocity of the boid

    Vector Align();						// Calculate the alignment force
    Vector Separate();					// Calculate the separation force
    Vector Cohesion();					// Calculate the cohesion force

    void Seek(Vector);					// Seek a new position
    void Contain();						// Contain the boids within the simulation area
};
