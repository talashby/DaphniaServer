

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "ParallelPhysics.h"
#include "vector"
#include "algorithm"
#include "array"
#include "thread"
#include "fstream"
#include "atomic"
#include "chrono"
#include "AdminProtocol.h"
#include "ServerProtocol.h"
#include "AdminTcp.h"
#include <assert.h>
#include "Observer.h"

#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#undef TEXT
#include <windows.h>
#include <winsock2.h>
#undef min
#undef max

#pragma warning( disable : 4018)


namespace PPh
{

ParallelPhysics *s_parallelPhysicsInstance = nullptr;

std::vector< std::vector< std::vector<struct EtherCell> > > s_universe;
std::atomic<uint64_t> s_time = 0; // absolute universe time
std::atomic<int32_t> s_waitThreadsCount = 0; // thread synchronization variable
std::vector<BoxIntMath> s_threadSimulateBounds; // [minVector; maxVector)
std::atomic<bool> s_bNeedUpdateSimulationBoxes;

struct ObserverCell
{
	ObserverCell(Observer *observer, const VectorInt32Math &position) : m_observer(observer), m_position(position) {}
	Observer *m_observer;
	VectorInt32Math m_position;
};
std::vector<ObserverCell> s_observers;

// stats
uint64_t s_quantumOfTimePerSecond = 0;
#define HIGH_PRECISION_STATS 1
std::vector<uint64_t> s_timingsUniverseThreads;
std::vector<uint64_t> s_TickTimeNsAverageUniverseThreads;
uint64_t s_timingsObserverThread;
uint64_t s_TickTimeNsAverageObserverThread;

struct Photon
{
	Photon() = default;
	explicit Photon(const OrientationVectorMath &orientation) : m_orientation(orientation)
	{}
	EtherColor m_color;
	OrientationVectorMath m_orientation;
	PhotonParam m_param;
};

struct EtherCell
{
	EtherCell() = default;
	explicit EtherCell(EtherType::EEtherType type) : m_type(type)
	{}
	int32_t m_type;
	EtherColor m_color;
	std::array <std::array<Photon, 26>, 2> m_photons;
};

bool ParallelPhysics::Init(const VectorInt32Math &universeSize, uint8_t threadsCount)
{
	if (0 < universeSize.m_posX && 0 < universeSize.m_posY && 0 < universeSize.m_posZ)
	{
		OrientationVectorMath::InitRandom();
		s_universe.resize(universeSize.m_posX);
		for (auto &itY : s_universe)
		{
			itY.resize(universeSize.m_posY);
			for (auto &itZ : itY)
			{
				itZ.resize(0);
				EtherCell cell(EtherType::Space);
				cell.m_color = EtherColor::ZeroColor;
				{
					for (int ii = 0; ii < cell.m_photons[0].size(); ++ii)
					{
						cell.m_photons[0][ii].m_color.m_colorA = 0;
					}
				}
				{
					for (int ii = 0; ii < cell.m_photons[1].size(); ++ii)
					{
						cell.m_photons[1][ii].m_color.m_colorA = 0;
					}
				}
				itZ.resize(universeSize.m_posZ, cell);
			}
		}
		if (s_parallelPhysicsInstance)
		{
			delete s_parallelPhysicsInstance;
		}
		s_parallelPhysicsInstance = new ParallelPhysics();
		GetInstance()->m_universeSize = universeSize;
		if (0 == threadsCount)
		{
			GetInstance()->m_bSimulateNearObserver = true;
			GetInstance()->m_threadsCount = 4;
		}
		else
		{
			GetInstance()->m_threadsCount = threadsCount;
		}
#ifdef HIGH_PRECISION_STATS
		s_timingsUniverseThreads.resize(GetInstance()->m_threadsCount);
		s_TickTimeNsAverageUniverseThreads.resize(GetInstance()->m_threadsCount);
#endif
		// fill bounds

		s_threadSimulateBounds.resize(GetInstance()->m_threadsCount);
		int32_t lengthForThread = GetInstance()->m_universeSize.m_posX / GetInstance()->m_threadsCount;
		int32_t lengthForThreadRemain = GetInstance()->m_universeSize.m_posX - lengthForThread * GetInstance()->m_threadsCount;
		int32_t beginX = 0;
		for (int ii = 0; ii < GetInstance()->m_threadsCount; ++ii)
		{
			int32_t lengthX = lengthForThread;
			if (0 < lengthForThreadRemain)
			{
				++lengthX;
				--lengthForThreadRemain;
			}
			int32_t endX = beginX + lengthX;
			s_threadSimulateBounds[ii].m_minVector = VectorInt32Math(beginX, 0, 0);
			s_threadSimulateBounds[ii].m_maxVector = VectorInt32Math(endX, GetInstance()->m_universeSize.m_posY, GetInstance()->m_universeSize.m_posZ);
			beginX = endX;
		}

		static std::thread s_adminTcpThread;
		s_adminTcpThread = std::thread(AdminTcpThread);
		return true;
	}
	return false;
}

bool ParallelPhysics::SaveUniverse(const std::string &fileName)
{
	std::ofstream myfile;
	myfile.open(fileName);
	if (myfile.is_open())
	{
		for (int32_t posX = 0; posX < s_universe.size(); ++posX)
		{
			for (int32_t posY = 0; posY < s_universe[posX].size(); ++posY)
			{
				for (int32_t posZ = 0; posZ < s_universe[posX][posY].size(); ++posZ)
				{
					myfile << (uint8_t)s_universe[posX][posY][posZ].m_type;
				}
			}
		}
		myfile.close();
		return true;
	}
	return false;
}

bool ParallelPhysics::LoadUniverse(const std::string &fileName)
{
	std::ifstream myfile(fileName);
	if (myfile.is_open())
	{
		for (int32_t posX = 0; posX < s_universe.size(); ++posX)
		{
			for (int32_t posY = 0; posY < s_universe[posX].size(); ++posY)
			{
				for (int32_t posZ = 0; posZ < s_universe[posX][posY].size(); ++posZ)
				{
					char c = myfile.get();
					if (!myfile.good())
					{
						return false;
					}
					EtherCell &cell = s_universe[posX][posY][posZ];
					cell.m_type = (EtherType::EEtherType)c;
					if (cell.m_type == EtherType::Crumb)
					{
						std::array<EtherColor, 4> Colors = {EtherColor(255,0,0), EtherColor(0,255,0), EtherColor(0,0,255), EtherColor(255,255,0)};
						cell.m_color = Colors[Rand32(4)];
					}
					else if (cell.m_type == EtherType::Block)
					{
						constexpr uint8_t blockGrayColor = 50;
						cell.m_color = EtherColor(blockGrayColor, blockGrayColor, blockGrayColor);
					}
				}
			}
		}
		myfile.close();
		return true;
	}
	return false;
}

void UniverseThread(int32_t threadNum, bool *isSimulationRunning)
{
	while (*isSimulationRunning)
	{
#ifdef HIGH_PRECISION_STATS
		auto beginTime = std::chrono::high_resolution_clock::now();
#endif
		int isTimeOdd = s_time % 2;

		for (int32_t posX = s_threadSimulateBounds[threadNum].m_minVector.m_posX; posX < s_threadSimulateBounds[threadNum].m_maxVector.m_posX; ++posX)
		{
			for (int32_t posY = s_threadSimulateBounds[threadNum].m_minVector.m_posY; posY < s_threadSimulateBounds[threadNum].m_maxVector.m_posY; ++posY)
			{
				for (int32_t posZ = s_threadSimulateBounds[threadNum].m_minVector.m_posZ; posZ < s_threadSimulateBounds[threadNum].m_maxVector.m_posZ; ++posZ)
				{
					EtherCell &cell = s_universe[posX][posY][posZ];
					if (cell.m_type == EtherType::Observer)
					{
						continue;
					}
					for (int ii = 0; ii < cell.m_photons[isTimeOdd].size(); ++ii)
					{
						Photon &photon = cell.m_photons[isTimeOdd][ii];
						if (photon.m_color.m_colorA != 0)
						{
							if (cell.m_type == EtherType::Crumb || cell.m_type == EtherType::Block)
							{
								photon.m_orientation *= -1;
								uint8_t tmpA = photon.m_color.m_colorA;
								photon.m_color = cell.m_color;
								photon.m_color.m_colorA = tmpA;
								if (cell.m_type == EtherType::Crumb)
								{
									volatile int ttt = 0;
								}
							}
    						if (photon.m_color.m_colorA > 10)
							{
								photon.m_color.m_colorA -= 10;
								bool result = ParallelPhysics::GetInstance()->EmitPhoton({ posX, posY, posZ }, photon);
								//if (result)
								{
									photon.m_color = EtherColor::ZeroColor;
								}
							}
							else
							{
								photon.m_color = EtherColor::ZeroColor;
							}
						}
					}
				}
			}
		}
#ifdef HIGH_PRECISION_STATS
		auto endTime = std::chrono::high_resolution_clock::now();
		auto dif = endTime - beginTime;
		s_timingsUniverseThreads[threadNum] += std::chrono::duration_cast<std::chrono::nanoseconds>(dif).count();
#endif
		--s_waitThreadsCount;
		while (s_time % 2 == isTimeOdd)
		{
		}
	}
	--s_waitThreadsCount;
}

ParallelPhysics* ParallelPhysics::GetInstance()
{
	return s_parallelPhysicsInstance;
}

const VectorInt32Math & ParallelPhysics::GetUniverseSize() const
{
	return m_universeSize;
}

void ParallelPhysics::AdjustSimulationBoxes()
{
	constexpr int32_t SIMULATION_SIZE = 8;

	VectorInt32Math observerPos = Observer::GetInstance()->GetPosition();

	VectorInt32Math boundsMin;
	{
		VectorInt32Math boundSize(SIMULATION_SIZE, SIMULATION_SIZE, SIMULATION_SIZE);
		const VectorInt32Math &orientMinChanger = Observer::GetInstance()->GetOrientMinChanger();
		boundSize.m_posX = std::min(boundSize.m_posX, orientMinChanger.m_posX);
		boundSize.m_posY = std::min(boundSize.m_posY, orientMinChanger.m_posY);
		boundSize.m_posZ = std::min(boundSize.m_posZ, orientMinChanger.m_posZ);
		boundsMin = observerPos - boundSize;
		AdjustSizeByBounds(boundsMin);
	}

	VectorInt32Math boundsMax;
	{
		VectorInt32Math boundSize(SIMULATION_SIZE, SIMULATION_SIZE, SIMULATION_SIZE);
		const VectorInt32Math &orientMaxChanger = Observer::GetInstance()->GetOrientMaxChanger();
		boundSize.m_posX = std::min(boundSize.m_posX, orientMaxChanger.m_posX);
		boundSize.m_posY = std::min(boundSize.m_posY, orientMaxChanger.m_posY);
		boundSize.m_posZ = std::min(boundSize.m_posZ, orientMaxChanger.m_posZ);
		boundsMax = observerPos + boundSize + VectorInt32Math::OneVector; // [minVector; maxVector)
		AdjustSizeByBounds(boundsMax);
	}
	
	int32_t lengthX = boundsMax.m_posX - boundsMin.m_posX;
	int32_t partX = lengthX / 4;
	int32_t remain = lengthX - partX * 4;

	int32_t posXBegin = boundsMin.m_posX;
	for (int ii = 0; ii < m_threadsCount; ++ii)
	{
		int32_t posXEnd = posXBegin + partX;
		if (0 < remain)
		{
			++posXEnd;
			--remain;
		}
		s_threadSimulateBounds[ii] = BoxIntMath({ posXBegin, boundsMin.m_posY, boundsMin.m_posZ }, { posXEnd, boundsMax.m_posY, boundsMax.m_posZ });
		posXBegin = posXEnd;
	}
	s_bNeedUpdateSimulationBoxes = false;
}

std::thread s_simulationThread;
void ParallelPhysics::StartSimulation()
{
	
	s_observers.push_back(ObserverCell(new Observer(0), )
	// init sockets
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	// thread
	//s_simulationThread = std::thread([this]() {
		m_isSimulationRunning = true;

		// UDP server
		SOCKET socketS;
		struct sockaddr_in local;
		local.sin_family = AF_INET;
		local.sin_port = htons(CLIENT_UDP_PORT_START);
		local.sin_addr.s_addr = INADDR_ANY;
		socketS = socket(AF_INET, SOCK_DGRAM, 0);
		bind(socketS, (sockaddr*)&local, sizeof(local));
		u_long mode = 1;  // 1 to enable non-blocking socket
		ioctlsocket(socketS, FIONBIO, &mode);
		
		// threads
		std::vector<std::thread> threads;
		threads.resize(m_threadsCount);

		s_waitThreadsCount = m_threadsCount;
		s_waitThreadsCount = m_threadsCount + 1; // universe threads and observers thread
		std::thread observersThread = std::thread([this]()
		{
			while (m_isSimulationRunning)
			{
#ifdef HIGH_PRECISION_STATS
				auto beginTime = std::chrono::high_resolution_clock::now();
#endif
				int32_t isTimeOdd = s_time % 2;
				Observer::GetInstance()->PPhTick();
#ifdef HIGH_PRECISION_STATS
				auto endTime = std::chrono::high_resolution_clock::now();
				s_timingsObserverThread += std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - beginTime).count();
#endif
				--s_waitThreadsCount;
				while (s_time % 2 == isTimeOdd)
				{
				}
			}
			--s_waitThreadsCount;
		});

		if (m_bSimulateNearObserver)
		{
			AdjustSimulationBoxes();
		}
		for (int ii = 0; ii < m_threadsCount; ++ii)
		{
			threads[ii] = std::thread(UniverseThread, ii, &m_isSimulationRunning);
		}

		int64_t lastTime = GetTimeMs();
		uint64_t lastTimeUniverse = 0;
		while (m_isSimulationRunning)
		{
			while (s_waitThreadsCount)
			{
			}
			s_waitThreadsCount = m_threadsCount;
			//s_waitThreadsCount = m_threadsCount + 1; // universe threads and observer thread
			if (Observer::GetInstance()->GetPosition() != Observer::GetInstance()->GetNewPosition())
			{
				VectorInt32Math newPos = Observer::GetInstance()->GetNewPosition();
				EtherCell &cell = s_universe[newPos.m_posX][newPos.m_posY][newPos.m_posZ];
				if (cell.m_type == EtherType::Crumb)
				{
					Observer::GetInstance()->IncEatenCrumb(newPos);
				}
				GetInstance()->InitEtherCell(newPos, EtherType::Observer);
				GetInstance()->InitEtherCell(Observer::GetInstance()->GetPosition(), EtherType::Space);
				Observer::GetInstance()->SetPosition(newPos);
				SetNeedUpdateSimulationBoxes();
			}
			if (m_bSimulateNearObserver && s_bNeedUpdateSimulationBoxes)
			{
				AdjustSimulationBoxes();
			}
			
			if (GetTimeMs() - lastTime >= 1000)
			{
				s_quantumOfTimePerSecond = s_time - lastTimeUniverse;
#ifdef HIGH_PRECISION_STATS
				for (int ii = 0; ii < s_timingsUniverseThreads.size(); ++ii)
				{
					if (s_timingsUniverseThreads[ii] > 0)
					{
						s_TickTimeNsAverageUniverseThreads[ii] = s_timingsUniverseThreads[ii] / s_quantumOfTimePerSecond;
						s_timingsUniverseThreads[ii] = 0;
					}
				}
				if (s_timingsObserverThread > 0)
				{
					s_TickTimeNsAverageObserverThread = s_timingsObserverThread / s_quantumOfTimePerSecond;
					s_timingsObserverThread = 0;
				}
#endif
				lastTime = GetTimeMs();
				lastTimeUniverse = s_time;
			}
			char buffer[64];
			struct sockaddr_in from;
			int fromlen = sizeof(from);
			bool isStateAlreadySent = false;
			while (recvfrom(socketS, buffer, sizeof(buffer), 0, (sockaddr*)&from, &fromlen) > 0)
			{
				if (QueryMessage<MsgGetState>(buffer))
				{
					if (isStateAlreadySent)
					{
						continue;
					}
					isStateAlreadySent = true;
					MsgGetStateResponse msgSendState;
					msgSendState.m_time = s_time;
					
					sendto(socketS, msgSendState.GetBuffer(), sizeof(msgSendState), 0, (sockaddr*)&from, fromlen);
					// receive photons back
					int isTimeOdd = (s_time+1) % 2;
					auto position = Observer::GetInstance()->GetPosition();
					EtherCell &cell = s_universe[position.m_posX][position.m_posY][position.m_posZ];
					for (int ii = 0; ii < cell.m_photons[isTimeOdd].size(); ++ii)
					{
						Photon &photon = cell.m_photons[isTimeOdd][ii];
						if (photon.m_color.m_colorA > 0)
						{
							int8_t posY = photon.m_param / OBSERVER_EYE_SIZE;
							int8_t posX = photon.m_param - posY * OBSERVER_EYE_SIZE;
							MsgSendPhoton msgSendPhoton;
							msgSendPhoton.m_color = photon.m_color;
							msgSendPhoton.m_posX = posX;
							msgSendPhoton.m_posY = posY;
							sendto(socketS, msgSendPhoton.GetBuffer(), sizeof(msgSendPhoton), 0, (sockaddr*)&from, fromlen);
							photon.m_color = EtherColor::ZeroColor;
						}
					}
				}
				else if (auto *msg = QueryMessage<MsgGetStateExt>(buffer))
				{
					MsgGetStateExtResponse msgSendState;
					msgSendState.m_latitude = Observer::GetInstance()->m_latitude;
					msgSendState.m_longitude = Observer::GetInstance()->m_longitude;
					msgSendState.m_pos = Observer::GetInstance()->GetPosition();
					msgSendState.m_movingProgress = Observer::GetInstance()->m_movingProgress;
					msgSendState.m_eatenCrumbNum = Observer::GetInstance()->m_eatenCrumbNum;
					msgSendState.m_eatenCrumbPos = Observer::GetInstance()->m_eatenCrumbPos;

					sendto(socketS, msgSendState.GetBuffer(), sizeof(msgSendState), 0, (sockaddr*)&from, fromlen);
				}
				else if (auto *msg = QueryMessage<MsgMoveForward>(buffer))
				{
					Observer::GetInstance()->MoveForward(msg->m_value);
				}
				else if (auto *msg = QueryMessage<MsgMoveBackward>(buffer))
				{
					Observer::GetInstance()->MoveBackward(msg->m_value);
				}
				else if (auto *msg = QueryMessage<MsgRotateLeft>(buffer))
				{
					Observer::GetInstance()->RotateLeft(msg->m_value);
				}
				else if (auto *msg = QueryMessage<MsgRotateRight>(buffer))
				{
					Observer::GetInstance()->RotateRight(msg->m_value);
				}
				else if (auto *msg = QueryMessage<MsgRotateUp>(buffer))
				{
					Observer::GetInstance()->RotateUp(msg->m_value);
				}
				else if (auto *msg = QueryMessage<MsgRotateDown>(buffer))
				{
					Observer::GetInstance()->RotateDown(msg->m_value);
				}
				}
			Observer::GetInstance()->Echolocation();
			++s_time;
		}
//		observerThread.join();
		for (int ii = 0; ii < m_threadsCount; ++ii)
		{
			threads[ii].join();
		}
		closesocket(socketS);
	//});
}

void ParallelPhysics::StopSimulation()
{
	m_isSimulationRunning = false;
	s_simulationThread.join();
}

bool ParallelPhysics::IsSimulationRunning() const
{
	return m_isSimulationRunning;
}

ParallelPhysics::ParallelPhysics()
{}

int32_t ParallelPhysics::GetCellPhotonIndex(const VectorInt32Math &unitVector)
{
	int32_t index = (unitVector.m_posX + 1) * 9 + (unitVector.m_posY + 1) * 3 + (unitVector.m_posZ + 1);
	if (index > 13)
	{
		--index;
	}
	return index;
}

bool ParallelPhysics::IsPosInBounds(const VectorInt32Math &pos)
{
	const VectorInt32Math &size = GetUniverseSize();
	if (pos.m_posX < 0 || pos.m_posY < 0 || pos.m_posZ < 0)
	{
		return false;
	}
	if (pos.m_posX >= size.m_posX || pos.m_posY >= size.m_posY || pos.m_posZ >= size.m_posZ)
	{
		return false;
	}
	return true;
}

bool ParallelPhysics::GetNextCrumb(VectorInt32Math & outCrumbPos, EtherColor & outCrumbColor)
{
	static int32_t s_posX = 0;
	static int32_t s_posY = 0;
	static int32_t s_posZ = 0;
	bool bResult = false;
	for (; s_posX < s_universe.size(); ++s_posX, s_posY=0)
	{
		for (; s_posY < s_universe[s_posX].size(); ++s_posY, s_posZ=0)
		{
			for (; s_posZ < s_universe[s_posX][s_posY].size(); ++s_posZ)
			{
				if (bResult)
				{
					break;
				}
				auto &cell = s_universe[s_posX][s_posY][s_posZ];
				if (cell.m_type == EtherType::Crumb)
				{
					outCrumbPos = VectorInt32Math(s_posX, s_posY, s_posZ);
					outCrumbColor = cell.m_color;
					bResult = true;
				}
			}
			if (bResult)
			{
				break;
			}
		}
		if (bResult)
		{
			break;
		}
	}
	if (!bResult)
	{
		s_posX = 0;
		s_posY = 0;
		s_posZ = 0;
	}
	return bResult;
}

void ParallelPhysics::AdjustSizeByBounds(VectorInt32Math &size)
{
	const VectorInt32Math &universeSize = GetUniverseSize();
	size.m_posX = std::max(0, size.m_posX);
	size.m_posY = std::max(0, size.m_posY);
	size.m_posZ = std::max(0, size.m_posZ);

	size.m_posX = std::min(universeSize.m_posX, size.m_posX);
	size.m_posY = std::min(universeSize.m_posY, size.m_posY);
	size.m_posZ = std::min(universeSize.m_posZ, size.m_posZ);
}

PPh::VectorInt32Math ParallelPhysics::GetRandomEmptyCell() const
{
	for (int ii=0; ii<10000; ++ii)
	{
		int32_t posX = Rand32(m_universeSize.m_posX);
		int32_t posY = Rand32(m_universeSize.m_posY);
		int32_t posZ = Rand32(m_universeSize.m_posZ);
		if (s_universe[posX][posY][posZ].m_type == EtherType::Space)
		{
			return VectorInt32Math(posX, posY, posZ);
		}
	}
	assert(false);
	return VectorInt32Math::ZeroVector;
}

bool ParallelPhysics::InitEtherCell(const VectorInt32Math &pos, EtherType::EEtherType type, const EtherColor &color)
{
	if (s_universe.size() > pos.m_posX)
	{
		if (s_universe[pos.m_posX].size() > pos.m_posY)
		{
			if (s_universe[pos.m_posX][pos.m_posY].size() > pos.m_posZ)
			{
				EtherCell &cell = s_universe[pos.m_posX][pos.m_posY][pos.m_posZ];
				cell.m_type = type;
				cell.m_color = color;
				for (int ii = 0; ii < cell.m_photons[0].size(); ++ii)
				{
					Photon &photon = cell.m_photons[0][ii];
					photon.m_color = EtherColor::ZeroColor;
				}
				for (int ii = 0; ii < cell.m_photons[1].size(); ++ii)
				{
					Photon &photon = cell.m_photons[1][ii];
					photon.m_color = EtherColor::ZeroColor;
				}
				return true;
			}
		}
	}
	return false;
}

VectorInt32Math CalculatePositionShift(const VectorInt32Math &pos, const OrientationVectorMath &orient)
{
	VectorInt32Math unitVector = VectorInt32Math::ZeroVector;
	if (std::abs(orient.m_posX) >= OrientationVectorMath::GetRandomNumber())
	{
		unitVector.m_posX = Sign(orient.m_posX);
	}
	if (std::abs(orient.m_posY) >= OrientationVectorMath::GetRandomNumber())
	{
		unitVector.m_posY = Sign(orient.m_posY);
	}
	if (std::abs(orient.m_posZ) >= OrientationVectorMath::GetRandomNumber())
	{
		unitVector.m_posZ = Sign(orient.m_posZ);
	}

	return unitVector;
}

bool ParallelPhysics::EmitPhoton(const VectorInt32Math &pos, const Photon &photon)
{
	VectorInt32Math unitVector = CalculatePositionShift(pos, photon.m_orientation);
	VectorInt32Math nextPos = pos + unitVector;
	if (IsPosInBounds(nextPos))
	{
		int isTimeOdd = (s_time + 1) % 2; // will be handle on next quantum of time

		EtherCell &cell = s_universe[nextPos.m_posX][nextPos.m_posY][nextPos.m_posZ];
		int32_t cellPhotonIndex = GetCellPhotonIndex(unitVector);
		Photon &photonCell = cell.m_photons[isTimeOdd][cellPhotonIndex];
		if (photonCell.m_color.m_colorA > 0)
		{
			return false;
		}
		photonCell = photon;
	}
	
	return true;
}

void ParallelPhysics::SetNeedUpdateSimulationBoxes()
{
	s_bNeedUpdateSimulationBoxes = true;
}

uint64_t ParallelPhysics::GetFPS()
{
	return s_quantumOfTimePerSecond;
}

bool ParallelPhysics::IsHighPrecisionStatsEnabled()
{
#ifdef HIGH_PRECISION_STATS
	return true;
#else
	return false;
#endif
}

uint64_t ParallelPhysics::GetTickTimeNsObserverThread()
{
	return s_TickTimeNsAverageObserverThread;
}

std::vector<uint64_t> ParallelPhysics::GetTickTimeNsUniverseThreads()
{
	return s_TickTimeNsAverageUniverseThreads;
}


}