
#undef UNICODE

#include "ParallelPhysics.h"


// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

int __cdecl main(void)
{
	PPh::ParallelPhysics::Init(PPh::VectorInt32Math(), 0);
	PPh::ParallelPhysics::GetInstance()->StartSimulation();
	return 0;
}
