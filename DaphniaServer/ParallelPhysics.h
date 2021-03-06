#pragma once

#include "PPhHelpers.h"
#include "memory"
#include "array"
#include "vector"

namespace PPh
{
// Constants
constexpr bool IS_DAPHNIA_BIG = true; // big Daphnia is 3x3x3

// Typedefs
typedef std::array<Photon, 26> EtherCellPhotonArray;

// Forward declarations
class Observer;
class MsgBase;

namespace ParallelPhysics
{
	bool Init(const VectorInt32Math &universeSize, uint8_t threadsCount, uint32_t universeScale); // returns true if success. threadsCount 0 means simulate near observer
	uint32_t GetUniverseScale();
	bool SaveUniverse(const std::string &fileName);
	bool LoadUniverse(const std::string &fileName);

	void StartSimulation();
	void StopSimulation();
	bool IsSimulationRunning();
/////////////////
//// For AdminTcp
	bool GetNextCrumb(VectorInt32Math &outCrumbPos, EtherColor &outCrumbColor);
	void SetAdminObserverId(uint64_t observerId);

/////////////////
//// For Observer
	void SetNeedUpdateSimulationBoxes();
	bool EmitEcholocationPhoton(const Observer *observer, const OrientationVectorMath &orientation, PhotonParam param);
	const char* RecvClientMsg(const Observer *observer); // returns nullptr if error occur
	void SendClientMsg(const Observer *observer, const MsgBase &msg, int32_t msgSize);
	void HandleOtherObserversPhotons(const Observer *observer); // should be called from observers thread
	EtherCellPhotonArray& GetReceivedPhotons(const Observer *observer);
	EtherCellPhotonArray& GetReceivedPhotonsForBigDaphnia(const Observer *observer, uint32_t index /*0-26*/);
	VectorInt32Math GetObserverPosition(const Observer *observer);
	// Stats
	uint32_t GetFPS();
	bool IsHighPrecisionStatsEnabled();
	uint32_t GetTickTimeMusObserverThread(); // average tick time in microseconds
	std::vector<uint32_t> GetTickTimeMusUniverseThreads(); // average tick time in microseconds
};

}
