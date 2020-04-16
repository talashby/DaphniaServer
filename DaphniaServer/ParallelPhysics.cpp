

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
VectorInt32Math CalculatePositionShift(const VectorInt32Math &pos, const OrientationVectorMath &orient);

ParallelPhysics *s_parallelPhysicsInstance = nullptr;

std::vector< std::vector< std::vector<struct EtherCell> > > s_universe;
std::atomic<uint64_t> s_time = 0; // absolute universe time
std::atomic<int32_t> s_waitThreadsCount = 0; // thread synchronization variable
std::vector<BoxIntMath> s_threadSimulateBounds; // [minVector; maxVector)
std::atomic<bool> s_bNeedUpdateSimulationBoxes;

struct ObserverCell
{
	ObserverCell(Observer *observer, const VectorInt32Math &position, SOCKET socket, const sockaddr_in &clientAddr) :
		m_observer(observer), m_position(position), m_socket(socket), m_clientAddr(clientAddr) {}
	Observer *m_observer;
	VectorInt32Math m_position; // universe position
	SOCKET m_socket; // server socket for client
	struct sockaddr_in m_clientAddr; // client ip address
};

std::vector<ObserverCell> s_observers;

// stats
uint64_t s_quantumOfTimePerSecond = 0;
#define HIGH_PRECISION_STATS 1
std::vector<uint64_t> s_timingsUniverseThreads;
std::vector<uint64_t> s_TickTimeNsAverageUniverseThreads;
uint64_t s_timingsObserverThread;
uint64_t s_TickTimeNsAverageObserverThread;

struct EtherCell
{
	EtherCell() = default;
	explicit EtherCell(EtherType::EEtherType type) : m_type(type)
	{}
	int32_t m_type;
	EtherColor m_color;
	std::array <EtherCellPhotonArray, 2> m_photons;
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
	if (!s_observers.size())
	{
		return;
	}
	constexpr int32_t SIMULATION_SIZE = 8;

	const Observer *observer = s_observers[0].m_observer;
	VectorInt32Math observerPos = s_observers[0].m_position;

	VectorInt32Math boundsMin;
	{
		VectorInt32Math boundSize(SIMULATION_SIZE, SIMULATION_SIZE, SIMULATION_SIZE);
		const VectorInt32Math &orientMinChanger = observer->GetOrientMinChanger();
		boundSize.m_posX = std::min(boundSize.m_posX, orientMinChanger.m_posX);
		boundSize.m_posY = std::min(boundSize.m_posY, orientMinChanger.m_posY);
		boundSize.m_posZ = std::min(boundSize.m_posZ, orientMinChanger.m_posZ);
		boundsMin = observerPos - boundSize;
		AdjustSizeByBounds(boundsMin);
	}

	VectorInt32Math boundsMax;
	{
		VectorInt32Math boundSize(SIMULATION_SIZE, SIMULATION_SIZE, SIMULATION_SIZE);
		const VectorInt32Math &orientMaxChanger = observer->GetOrientMaxChanger();
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
SOCKET s_socketForNewClient = -1;
void CreateSocketForNewClient()
{
	if (s_observers.size() < MAX_CLIENTS)
	{
		// UDP
		SOCKET socketS;
		struct sockaddr_in local;
		local.sin_family = AF_INET;
		local.sin_port = htons(CLIENT_UDP_PORT_START + (u_short)s_observers.size());
		local.sin_addr.s_addr = INADDR_ANY;
		socketS = socket(AF_INET, SOCK_DGRAM, 0);
		bind(socketS, (sockaddr*)&local, sizeof(local));
		u_long mode = 1;  // 1 to enable non-blocking socket
		ioctlsocket(socketS, FIONBIO, &mode);
		s_socketForNewClient = socketS;
	}
	else
	{
		assert(s_socketForNewClient == -1);
	}
}

void ParallelPhysics::StartSimulation()
{
	// init sockets
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	CreateSocketForNewClient();
	// thread
	//s_simulationThread = std::thread([this]() {
		m_isSimulationRunning = true;


		
		// threads
		std::vector<std::thread> threads;
		threads.resize(m_threadsCount);

		s_waitThreadsCount = m_threadsCount + 1; // universe threads and observers thread
		std::thread observersThread = std::thread([this]()
		{
			while (m_isSimulationRunning)
			{
#ifdef HIGH_PRECISION_STATS
				auto beginTime = std::chrono::high_resolution_clock::now();
#endif
				if (s_socketForNewClient != -1)
				{

					char buffer[64];
					struct sockaddr_in from;
					int fromlen = sizeof(from);
					while (recvfrom(s_socketForNewClient, buffer, sizeof(buffer), 0, (sockaddr*)&from, &fromlen) > 0)
					{
						if (const MsgGetVersion *msg = QueryMessage<MsgGetVersion>(buffer))
						{
							MsgGetVersionResponse msgGetVersionResponse;
							msgGetVersionResponse.m_serverVersion = PROTOCOL_VERSION;
							sendto(s_socketForNewClient, msgGetVersionResponse.GetBuffer(), sizeof(msgGetVersionResponse), 0, (sockaddr*)&from, fromlen);
							if (msg->m_clientVersion == PROTOCOL_VERSION)
							{
								uint8_t observerIndex = (uint8_t)s_observers.size();
								s_observers.push_back(ObserverCell(new Observer(observerIndex), GetRandomEmptyCell(), s_socketForNewClient, from));
								InitEtherCell(s_observers.back().m_position, EtherType::Observer, EtherColor(255, 255, 255, observerIndex));
								s_socketForNewClient = -1;
								CreateSocketForNewClient();
							}
						}
					}
				}

				int32_t isTimeOdd = s_time % 2;
				for (auto &observer : s_observers)
				{
					observer.m_observer->PPhTick(s_time);
					ClearReceivedPhotons(observer.m_observer);
				}

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
			s_waitThreadsCount = m_threadsCount + 1; // universe threads and observers thread
			for (ObserverCell &observer : s_observers)
			{
				bool moveForward = observer.m_observer->GrabMoveForward();
				bool moveBackward = observer.m_observer->GrabMoveBackward();
				if (moveForward || moveBackward)
				{
					assert(s_observers.size() > observer.m_observer->m_index);
					VectorInt32Math pos = observer.m_position;
					OrientationVectorMath orient = observer.m_observer->GetOrientation();
					if (moveBackward)
					{
						orient *= -1;
					}
					VectorInt32Math unitVector = CalculatePositionShift(observer.m_position, orient);
					VectorInt32Math nextPos = pos + unitVector;
					if (ParallelPhysics::GetInstance()->IsPosInBounds(nextPos))
					{
						EtherCell &cell = s_universe[pos.m_posX][pos.m_posY][pos.m_posZ];
						EtherCell &nextCell = s_universe[nextPos.m_posX][nextPos.m_posY][nextPos.m_posZ];
						if (nextCell.m_type == EtherType::Space || nextCell.m_type == EtherType::Crumb)
						{
							if (nextCell.m_type == EtherType::Crumb)
							{
								observer.m_observer->IncEatenCrumb(nextPos);
							}
							observer.m_position = nextPos;
							nextCell.m_type = EtherType::Observer;
							nextCell.m_color = cell.m_color;
							cell.m_type = EtherType::Space;
						}
						SetNeedUpdateSimulationBoxes();
					}
				}
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
			++s_time;
		}
		observersThread.join();
		for (int ii = 0; ii < m_threadsCount; ++ii)
		{
			threads[ii].join();
		}
		if (s_socketForNewClient != -1)
		{
			closesocket(s_socketForNewClient);
		}
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

bool ParallelPhysics::EmitEcholocationPhoton(const Observer *observer, const OrientationVectorMath &orientation, PhotonParam param)
{
	assert(s_observers.size() > observer->m_index);
	const VectorInt32Math &pos = s_observers[observer->m_index].m_position;
	Photon photon(orientation);
	photon.m_param = param;
	photon.m_color.m_colorA = 255;
	return EmitPhoton(pos, photon);
}

const char* ParallelPhysics::RecvClientMsg(const Observer *observer)
{
	assert(s_observers.size() > observer->m_index);
	SOCKET socket = s_observers[observer->m_index].m_socket;
	struct sockaddr_in &from = s_observers[observer->m_index].m_clientAddr;

	static char buffer[64];
	struct sockaddr_in fromCur;
	int fromlen = sizeof(sockaddr_in);
	int result = recvfrom(socket, buffer, sizeof(buffer), 0, (sockaddr*)&fromCur, &fromlen);
	if (result > 0)
	{
		if (from.sin_addr.s_addr != fromCur.sin_addr.s_addr || from.sin_port != fromCur.sin_port)
		{
			MsgSocketBusyByAnotherObserver msg;
			msg.m_serverVersion = PROTOCOL_VERSION;
			sendto(socket, msg.GetBuffer(), sizeof(msg), 0, (sockaddr*)&fromCur, fromlen);
			return nullptr;
		}
		return &buffer[0];
	}

	return nullptr;
}

void ParallelPhysics::SendClientMsg(const Observer *observer, const MsgBase &msg, int32_t msgSize)
{
	assert(s_observers.size() > observer->m_index);
	SOCKET socket = s_observers[observer->m_index].m_socket;
	struct sockaddr_in &from = s_observers[observer->m_index].m_clientAddr;
	int fromlen = sizeof(sockaddr_in);

	sendto(socket, msg.GetBuffer(), msgSize, 0, (sockaddr*)&from, fromlen);
}

const EtherCellPhotonArray& ParallelPhysics::GetReceivedPhotons(const class Observer *observer)
{
	assert(s_observers.size() > observer->m_index);
	VectorInt32Math pos = s_observers[observer->m_index].m_position;
	const EtherCell &cell = s_universe[pos.m_posX][pos.m_posY][pos.m_posZ];
	int isTimeOdd = (s_time + 1) % 2;
	const EtherCellPhotonArray &photonArray = cell.m_photons[isTimeOdd];
	return photonArray;
}

PPh::VectorInt32Math ParallelPhysics::GetObserverPosition(const class Observer *observer)
{
	assert(s_observers.size() > observer->m_index);
	VectorInt32Math pos = s_observers[observer->m_index].m_position;
	return pos;
}

void ParallelPhysics::ClearReceivedPhotons(const Observer *observer)
{
	assert(s_observers.size() > observer->m_index);
	VectorInt32Math pos = s_observers[observer->m_index].m_position;
	EtherCell &cell = s_universe[pos.m_posX][pos.m_posY][pos.m_posZ];
	int isTimeOdd = (s_time + 1) % 2;
	EtherCellPhotonArray &photonArray = cell.m_photons[isTimeOdd];
	for (Photon &photon : photonArray)
	{
		photon.m_color.m_colorA = 0;
	}
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