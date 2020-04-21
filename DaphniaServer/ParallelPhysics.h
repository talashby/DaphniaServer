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

typedef std::array<Photon, 26> EtherCellPhotonArray;

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

	static uint32_t GetFPS();
	static bool IsHighPrecisionStatsEnabled();
	static uint32_t GetTickTimeMusObserverThread(); // average tick time in microseconds
	static std::vector<uint32_t> GetTickTimeMusUniverseThreads(); // average tick time in microseconds

	bool IsPosInBounds(const VectorInt32Math &pos);
	bool GetNextCrumb(VectorInt32Math &outCrumbPos, EtherColor &outCrumbColor);

	bool EmitEcholocationPhoton(const class Observer *observer, const OrientationVectorMath &orientation, PhotonParam param);
	bool EmitPhoton(const VectorInt32Math &pos, const struct Photon &photon); // TODO make private

	static const char* RecvClientMsg(const class Observer *observer); // returns nullptr if error occur
	static void SendClientMsg(const class Observer *observer, const class MsgBase &msg, int32_t msgSize);
	static void HandleOtherObserversPhotons(const class Observer *observer);
	static const EtherCellPhotonArray& GetReceivedPhotons(const class Observer *observer);
	static VectorInt32Math GetObserverPosition(const class Observer *observer);

private:
	ParallelPhysics();

	bool InitEtherCell(const VectorInt32Math &pos, EtherType::EEtherType type, const EtherColor &color = EtherColor()); // returns true if success
	static int32_t GetCellPhotonIndex(const VectorInt32Math &unitVector);
	void AdjustSimulationBoxes();
	void AdjustSizeByBounds(VectorInt32Math &size);
	VectorInt32Math GetRandomEmptyCell() const;
	static void ClearReceivedPhotons(const class Observer *observer);

	VectorInt32Math m_universeSize = VectorInt32Math::ZeroVector;
	uint8_t m_threadsCount = 1;
	bool m_bSimulateNearObserver = false;
	bool m_isSimulationRunning = false;
};

}
