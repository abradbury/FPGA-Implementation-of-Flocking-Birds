#include "boidCPU.h"

int main() {
	hls::stream<uint32> to_hw, from_hw;

	topleveltwo(to_hw, from_hw);

	return 0;
}
