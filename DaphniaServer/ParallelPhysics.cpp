

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
		local.sin_port = htons(CLIENT_UDP_PORT);
		local.sin_addr.s_addr = INADDR_ANY;
		socketS = socket(AF_INET, SOCK_DGRAM, 0);
		bind(socketS, (sockaddr*)&local, sizeof(local));
		u_long mode = 1;  // 1 to enable non-blocking socket
		ioctlsocket(socketS, FIONBIO, &mode);
		
		// threads
		std::vector<std::thread> threads;
		threads.resize(m_threadsCount);

		s_waitThreadsCount = m_threadsCount;
		//s_waitThreadsCount = m_threadsCount + 1; // universe threads and observer thread
/*		std::thread observerThread = std::thread([this]()
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
		});*/

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
					Observer::GetInstance()->IncEatenCrumb();
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
					MsgSendState msgSendState;
					msgSendState.m_time = s_time;
					msgSendState.m_latitude = Observer::GetInstance()->m_latitude;
					msgSendState.m_longitude = Observer::GetInstance()->m_longitude;
					if (Observer::GetInstance()->DecEatenCrumb())
					{
						msgSendState.m_isEatenCrumb = true;
					}
					else
					{
						msgSendState.m_isEatenCrumb = false;
					}
					
					sendto(socketS, msgSendState.GetBuffer(), sizeof(MsgSendState), 0, (sockaddr*)&from, fromlen);
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
	for (; s_posX < s_universe.size(); ++s_posX)
	{
		if (bResult)
		{
			break;
		}
		for (; s_posY < s_universe[s_posX].size(); ++s_posY)
		{
			if (bResult)
			{
				break;
			}
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
		}
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

Observer* s_observer = nullptr;

void Observer::Init(const VectorInt32Math &position, const SP_EyeState &eyeState)
{
	if (s_observer)
	{
		delete s_observer;
	}
	s_observer = new Observer();
	s_observer->m_position = position;
	s_observer->m_newPosition = position;
	//s_observer->m_eyeState = eyeState;
	//s_observer->CalculateOrientChangers(*eyeState);
	ParallelPhysics::GetInstance()->InitEtherCell(position, EtherType::Observer);
	s_observer->m_lastTextureUpdateTime = GetTimeMs();
	s_observer->m_latitude = 0;
	s_observer->m_longitude = 0;
	s_observer->CalculateEyeState();
}

PPh::Observer* Observer::GetInstance()
{
	return s_observer;
}

void Observer::Echolocation()
{
	const EyeArray &eyeArray = *m_eyeState;
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		Photon photon(eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(m_position, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		Photon photon(eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(m_position, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		Photon photon(eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(m_position, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		Photon photon(eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(m_position, photon);
	}
}

void Observer::ChangeOrientation(const SP_EyeState &eyeState)
{
	std::atomic_store(&m_newEyeState, eyeState);
}

SP_EyeColorArray Observer::GrabTexture()
{
	SP_EyeColorArray spEyeColorArrayOut;
	std::atomic_store(&spEyeColorArrayOut, m_spEyeColorArrayOut);
	SP_EyeColorArray spEyeColorArrayEmpty;
	std::atomic_store(&m_spEyeColorArrayOut, spEyeColorArrayEmpty);
	return spEyeColorArrayOut;
}

PPh::VectorInt32Math Observer::GetPosition() const
{
	return m_position;
}

void Observer::SetNewPosition(const VectorInt32Math &pos)
{
	m_newPosition = pos;
}

PPh::VectorInt32Math Observer::GetNewPosition() const
{
	return m_newPosition;
}

const VectorInt32Math& Observer::GetOrientMinChanger() const
{
	return m_orientMinChanger;
}

const VectorInt32Math& Observer::GetOrientMaxChanger() const
{
	return m_orientMaxChanger;
}

void Observer::CalculateEyeState()
{
	if (!m_eyeState)
	{
		m_eyeState = std::make_shared<PPh::EyeArray>();
	}
	PPh::EyeArray &eyeArray = *m_eyeState;

	float len = EYE_FOV / OBSERVER_EYE_SIZE;

	for (int32_t yy = 0; yy < OBSERVER_EYE_SIZE; ++yy)
	{
		for (int32_t xx = 0; xx < OBSERVER_EYE_SIZE; ++xx)
		{

			int16_t latitude = m_latitude + EYE_FOV * yy / OBSERVER_EYE_SIZE - EYE_FOV / 2;
			int16_t longitude = 0;
			int16_t longitudeShift = EYE_FOV * xx / OBSERVER_EYE_SIZE - EYE_FOV / 2;
			if (latitude < - 90 || latitude > 90)
			{
				latitude = Sign(latitude)*180 - latitude;
				longitude = m_longitude - longitudeShift;
				longitude = longitude < -179 ? 360 + longitude : longitude;
				longitude = longitude > 180 ? -360 + longitude : longitude;
				longitude = longitude - 180;
				longitude = longitude < -179 ? 360 + longitude : longitude;
			}
			else
			{
				longitude = m_longitude + longitudeShift;
				longitude = longitude < -179 ? 360 + longitude : longitude;
				longitude = longitude > 180 ? -360 + longitude : longitude;
			}

			float pi = 3.1415927410125732421875f;

			int16_t latitudeDownFactor = (int16_t)(abs(longitudeShift) * sinf(latitude * pi / 180));
			//int16_t latitudeDownFactor = abs(longitudeShift) * latitude / 180;
			latitude -= latitudeDownFactor;
			assert(latitude <= 90);
			assert(latitude >= -90);


			VectorFloatMath orientFloat;
			orientFloat.m_posX = cosf(latitude * pi / 180) * cosf(longitude * pi / 180);
			orientFloat.m_posY = cosf(latitude * pi / 180) * sinf(longitude * pi / 180);
			orientFloat.m_posZ = sinf(latitude * pi / 180);

			OrientationVectorMath orient  = MaximizePPhOrientation(orientFloat);
			eyeArray[yy][xx] = orient;
		}
	}
	CalculateOrientChangers(*m_eyeState);
}

void Observer::MoveForward(uint8_t value)
{
	auto movingProgressTmp = m_movingProgress;
	m_movingProgress += value;
	if (movingProgressTmp > m_movingProgress)
	{
		VectorInt32Math pos = GetPosition();
		VectorInt32Math unitVector = CalculatePositionShift(pos, GetOrientation());
		VectorInt32Math nextPos = pos + unitVector;
		if (ParallelPhysics::GetInstance()->IsPosInBounds(nextPos))
		{
			SetNewPosition(nextPos);
		}
	}
}

void Observer::MoveBackward(uint8_t value)
{
	auto movingProgressTmp = m_movingProgress;
	m_movingProgress -= value;
	if (movingProgressTmp < m_movingProgress)
	{
		VectorInt32Math pos = GetPosition();
		VectorInt32Math unitVector = CalculatePositionShift(pos, GetOrientation());
		VectorInt32Math nextPos = pos - unitVector;
		if (ParallelPhysics::GetInstance()->IsPosInBounds(nextPos))
		{
			SetNewPosition(nextPos);
		}
	}
}

void Observer::RotateLeft(uint8_t value)
{
	auto movingProgressTmp = m_longitudeProgress;
	m_longitudeProgress -= value;
	if (movingProgressTmp < m_longitudeProgress)
	{
		--m_longitude;
		if (m_longitude < -179)
		{
			m_longitude += 360;
		}
		CalculateEyeState();
	}
}

void Observer::RotateRight(uint8_t value)
{
	auto movingProgressTmp = m_longitudeProgress;
	m_longitudeProgress += value;
	if (movingProgressTmp > m_longitudeProgress)
	{
		++m_longitude;
		if (m_longitude > 180)
		{
			m_longitude -= 360;
		}
		CalculateEyeState();
	}
}

void Observer::RotateUp(uint8_t value)
{
	auto movingProgressTmp = m_latitudeProgress;
	m_latitudeProgress += value;
	if (movingProgressTmp > m_latitudeProgress)
	{
		++m_latitude;
		if (m_latitude > 90)
		{
			--m_latitude;
		}
		else
		{
			CalculateEyeState();
		}
	}
}

void Observer::RotateDown(uint8_t value)
{
	auto movingProgressTmp = m_latitudeProgress;
	m_latitudeProgress -= value;
	if (movingProgressTmp < m_latitudeProgress)
	{
		--m_latitude;
		if (m_latitude < -90)
		{
			++m_latitude;
		}
		else
		{
			CalculateEyeState();
		}
	}
}

void Observer::IncEatenCrumb()
{
	++m_eatenCrumb;
}

bool Observer::DecEatenCrumb()
{
	if (m_eatenCrumb > 0)
	{
		--m_eatenCrumb;
		return true;
	}
	return false;
}

OrientationVectorMath Observer::GetOrientation() const
{
	VectorFloatMath orientFloat;
	float pi = 3.1415927410125732421875f;
	orientFloat.m_posX = cosf(m_latitude * pi / 180) * cosf(m_longitude * pi / 180);
	orientFloat.m_posY = cosf(m_latitude * pi / 180) * sinf(m_longitude * pi / 180);
	orientFloat.m_posZ = sinf(m_latitude * pi / 180);

	OrientationVectorMath orient = MaximizePPhOrientation(orientFloat);
	return orient;
}

int32_t RoundToMinMaxPPhInt(float value)
{
	int32_t result = 0;
	if (value < 0)
	{
		result = (int32_t)(value - 0.5f);
	}
	else
	{
		result = (int32_t)(value + 0.5f);
	}

	return result;
}

/*int32_t FixFloatErrors(int32_t component, int32_t maxComponentValue)
{
	int32_t componentCorrect = component;
	if (std::abs(component) == maxComponentValue)
	{
		if (0 > component)
		{
			componentCorrect = PPh::OrientationVectorMath::PPH_INT_MIN;
		}
		else
		{
			componentCorrect = PPh::OrientationVectorMath::PPH_INT_MAX;
		}
	}
	return componentCorrect;
}*/

OrientationVectorMath Observer::MaximizePPhOrientation(const VectorFloatMath &orientationVector) const
{
	float maxComponent = std::max(std::max(std::abs(orientationVector.m_posX), std::abs(orientationVector.m_posY)), std::abs(orientationVector.m_posZ));
	float factor = 0;
	if (maxComponent > 0)
	{
		factor = OrientationVectorMath::PPH_INT_MAX / (float)maxComponent;
	}

	OrientationVectorMath pphOrientation(RoundToMinMaxPPhInt(orientationVector.m_posX*factor), RoundToMinMaxPPhInt(orientationVector.m_posY*factor),
		RoundToMinMaxPPhInt(orientationVector.m_posZ*factor));

	//int32_t maxPPhComponent = std::max(std::max(std::abs(pphOrientation.m_posX), std::abs(pphOrientation.m_posY)), std::abs(pphOrientation.m_posZ));
	//pphOrientation.m_posX = FixFloatErrors(pphOrientation.m_posX, maxPPhComponent);
	//pphOrientation.m_posY = FixFloatErrors(pphOrientation.m_posY, maxPPhComponent);
	//pphOrientation.m_posZ = FixFloatErrors(pphOrientation.m_posZ, maxPPhComponent);

	return pphOrientation;
}

void Observer::SetPosition(const VectorInt32Math &pos)
{
	m_position = pos;
}

void Observer::CalculateOrientChangers(const EyeArray &eyeArray)
{
	OrientationVectorMath orientMin(OrientationVectorMath::PPH_INT_MAX, OrientationVectorMath::PPH_INT_MAX, OrientationVectorMath::PPH_INT_MAX);
	OrientationVectorMath orientMax(OrientationVectorMath::PPH_INT_MIN, OrientationVectorMath::PPH_INT_MIN, OrientationVectorMath::PPH_INT_MIN);
	for (int yy = 0; yy < eyeArray.size(); ++yy)
	{
		for (int xx = 0; xx < eyeArray[yy].size(); ++xx)
		{
			orientMin.m_posX = std::min(orientMin.m_posX, eyeArray[yy][xx].m_posX);
			orientMin.m_posY = std::min(orientMin.m_posY, eyeArray[yy][xx].m_posY);
			orientMin.m_posZ = std::min(orientMin.m_posZ, eyeArray[yy][xx].m_posZ);
			orientMax.m_posX = std::max(orientMax.m_posX, eyeArray[yy][xx].m_posX);
			orientMax.m_posY = std::max(orientMax.m_posY, eyeArray[yy][xx].m_posY);
			orientMax.m_posZ = std::max(orientMax.m_posZ, eyeArray[yy][xx].m_posZ);
		}
	}

	m_orientMinChanger = VectorInt32Math(VectorInt32Math::PPH_INT_MAX, VectorInt32Math::PPH_INT_MAX, VectorInt32Math::PPH_INT_MAX);
	for (int ii = 0; ii < 3; ++ii)
	{
		if (0 <= orientMin.m_posArray[ii] && 0 <= orientMax.m_posArray[ii])
		{
			m_orientMinChanger.m_posArray[ii] = 0;
		}
	}

	m_orientMaxChanger = VectorInt32Math(VectorInt32Math::PPH_INT_MAX, VectorInt32Math::PPH_INT_MAX, VectorInt32Math::PPH_INT_MAX);
	for (int ii = 0; ii < 3; ++ii)
	{
		if (0 >= orientMin.m_posArray[ii] && 0 >= orientMax.m_posArray[ii])
		{
			m_orientMaxChanger.m_posArray[ii] = 0;
		}
	}
}

}