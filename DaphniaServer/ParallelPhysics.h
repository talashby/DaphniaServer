#pragma once

#include "PPhHelpers.h"
#include "memory"
#include "array"
#include "vector"

namespace PPh
{

struct EtherType
{
	enum EEtherType
	{
		Space = 0,
		Crumb,
		Block,
		Observer
	};
};

class ParallelPhysics
{
public:

	static bool Init(const VectorInt32Math &universeSize, uint8_t threadsCount); // returns true if success. threadsCount 0 means simulate near observer
	static bool SaveUniverse(const std::string &fileName);
	static bool LoadUniverse(const std::string &fileName);
	static ParallelPhysics* GetInstance();

	const VectorInt32Math & GetUniverseSize() const;

	void StartSimulation();
	void StopSimulation();
	bool IsSimulationRunning() const;

	static void SetNeedUpdateSimulationBoxes();

	static uint64_t GetFPS();
	static bool IsHighPrecisionStatsEnabled();
	static uint64_t GetTickTimeNsObserverThread(); // average tick time in nanoseconds
	static std::vector<uint64_t> GetTickTimeNsUniverseThreads(); // average tick time in nanoseconds

	bool IsPosInBounds(const VectorInt32Math &pos);
	bool GetNextCrumb(VectorInt32Math &outCrumbPos, EtherColor &outCrumbColor);

	bool EmitEcholocationPhoton(const class Observer *observer, const OrientationVectorMath &orientation, PhotonParam param);

private:
	ParallelPhysics();

	bool InitEtherCell(const VectorInt32Math &pos, EtherType::EEtherType type, const EtherColor &color = EtherColor()); // returns true if success
	static int32_t GetCellPhotonIndex(const VectorInt32Math &unitVector);
	void AdjustSimulationBoxes();
	void AdjustSizeByBounds(VectorInt32Math &size);
	VectorInt32Math GetRandomEmptyCell() const;
	bool EmitPhoton(const VectorInt32Math &pos, const struct Photon &photon);


	VectorInt32Math m_universeSize = VectorInt32Math::ZeroVector;
	uint8_t m_threadsCount = 1;
	bool m_bSimulateNearObserver = false;
	bool m_isSimulationRunning = false;
};

}
