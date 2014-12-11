#ifndef __TOPLEVEL_H_
#define __TOPLEVEL_H_

#include <stdio.h>
#include <stdlib.h>
#include <ap_int.h>
#include <hls_stream.h>

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;

typedef ap_uint<8> uint8;
typedef ap_int<8> int8;

//Prototypes
void topleveltwo(hls::stream<uint32> &input, hls::stream<uint32> &output);

#endif
