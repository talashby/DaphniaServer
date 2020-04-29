#pragma once

#include "PPhHelpers.h"
#include "memory"
#include "array"
#include "vector"

namespace PPh
{

typedef std::array<Photon, 26> EtherCellPhotonArray;
class Observer;
class MsgBase;

namespace ParallelPhysics
{
	bool Init(const VectorInt32Math &universeSize, uint8_t threadsCount); // returns true if success. threadsCount 0 means simulate near observer
	bool SaveUniverse(const std::string &fileName);
	bool LoadUniverse(const std::string &fileName);

	void StartSimulation();
	void StopSimulation();
	bool IsSimulationRunning();
/////////////////
//// For AdminTcp
	bool GetNextCrumb(VectorInt32Math &outCrumbPos, EtherColor &outCrumbColor);

/////////////////
//// For Observer
	void SetNeedUpdateSimulationBoxes();
	bool EmitEcholocationPhoton(const Observer *observer, const OrientationVectorMath &orientation, PhotonParam param);
	const char* RecvClientMsg(const Observer *observer); // returns nullptr if error occur
	void SendClientMsg(const Observer *observer, const MsgBase &msg, int32_t msgSize);
	void HandleOtherObserversPhotons(const Observer *observer); // should be called from observers thread
	const EtherCellPhotonArray& GetReceivedPhotons(const Observer *observer);
	VectorInt32Math GetObserverPosition(const Observer *observer);
	// Stats
	uint32_t GetFPS();
	bool IsHighPrecisionStatsEnabled();
	uint32_t GetTickTimeMusObserverThread(); // average tick time in microseconds
	std::vector<uint32_t> GetTickTimeMusUniverseThreads(); // average tick time in microseconds
};

}
