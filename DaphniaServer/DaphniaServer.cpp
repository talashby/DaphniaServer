
#undef UNICODE

#include "ParallelPhysics.h"
#include <iostream>


// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cout << "Not enough arguments";
	}
	PPh::VectorInt32Math size;
	size.m_posX = std::atoi(argv[0]);
	size.m_posY = std::atoi(argv[1]);
	size.m_posZ = std::atoi(argv[2]);

	PPh::ParallelPhysics::Init(size, 0);
	PPh::ParallelPhysics::GetInstance()->StartSimulation();
	return 0;
}
