
#undef UNICODE

#include "ParallelPhysics.h"
#include <iostream>


// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

int main(int argc, char** argv)
{
	if (argc < 5)
	{
		std::cout << "Not enough arguments";
		return 0;
	}
	PPh::VectorInt32Math size;
	size.m_posX = std::atoi(argv[1]);
	size.m_posY = std::atoi(argv[2]);
	size.m_posZ = std::atoi(argv[3]);

	PPh::ParallelPhysics::Init(size, 0);
	if (PPh::ParallelPhysics::LoadUniverse(argv[4]))
	{
		PPh::ParallelPhysics::StartSimulation();
	}
	return 0;
}
