
#undef UNICODE

#include "ParallelPhysics.h"
#include <iostream>


// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

int main(int argc, char** argv)
{
	if (argc < 7)
	{
		std::cout << "Not enough arguments";
		return 0;
	}
	PPh::VectorInt32Math size;
	size.m_posX = std::atoi(argv[1]);
	size.m_posY = std::atoi(argv[2]);
	size.m_posZ = std::atoi(argv[3]);

	printf("Initialization started.\n");
	PPh::ParallelPhysics::Init(size, std::atoi(argv[5]), std::atoi(argv[6]));
	printf("Loading Universe...\n");
	if (PPh::ParallelPhysics::LoadUniverse(argv[4]))
	{
		printf("Simulation started!\n");
		PPh::ParallelPhysics::StartSimulation();
	}
	else
	{
		printf("Loading failed\n");
	}
	return 0;
}
