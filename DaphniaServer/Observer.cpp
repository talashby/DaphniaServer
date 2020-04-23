
#include "Observer.h"
#include <assert.h>
#include "ServerProtocol.h"
#include <algorithm>

namespace PPh
{
Observer::Observer(int32_t index) : m_index(index)
{
	CalculateEyeState();
}

void Observer::PPhTick(uint64_t universeTime)
{
	bool isCalculateEyeStateNeeded = false;

	static_assert(MsgType::ClientToServerEnd <= 64);
	uint64_t receivedMessagesBitset = 0;

	ParallelPhysics::HandleOtherObserversPhotons(this);

	while (const char *buffer = ParallelPhysics::RecvClientMsg(this))
	{
		if (buffer[0] >= MsgType::ClientToServerEnd)
		{
			printf("Wrong message from client %d. Message type: %d.", m_index, buffer[0]);
			continue;
		}
		if (receivedMessagesBitset & (1ULL << (buffer[0]))) // test bit
		{
			if (MsgType::GetState == buffer[0])
			{
				++m_skippedGetStateAfterLastSendStatistics;
			}
			continue; // handle one message of same type in a tick
		}
		receivedMessagesBitset |= 1ULL << buffer[0]; // set bit
		switch ((MsgType::MsgType)buffer[0])
		{
		case MsgType::CheckVersion:
		{
			MsgCheckVersionResponse msgCheckVersionResponse;
			msgCheckVersionResponse.m_observerId = reinterpret_cast<uint64_t>(this);
			msgCheckVersionResponse.m_serverVersion = PROTOCOL_VERSION;
			ParallelPhysics::SendClientMsg(this, msgCheckVersionResponse, sizeof(msgCheckVersionResponse));
		}
		break;
		case MsgType::GetStatistics:
		{
			MsgGetStatisticsResponse msg;
			msg.m_fps = ParallelPhysics::GetFPS();
			msg.m_observerThreadTickTime = ParallelPhysics::GetTickTimeMusObserverThread();

			const std::vector<uint32_t> &universeThreadsTimings = ParallelPhysics::GetTickTimeMusUniverseThreads();
			if (universeThreadsTimings.size() > 0)
			{
				uint32_t timeMusMin = universeThreadsTimings[0];
				uint32_t timeMusMax = universeThreadsTimings[0];
				for (int32_t ii=1; ii < universeThreadsTimings.size(); ++ii)
				{
					timeMusMin = std::min(timeMusMin, universeThreadsTimings[ii]);
					timeMusMax = std::max(timeMusMax, universeThreadsTimings[ii]);
				}
				msg.m_universeThreadMinTickTime = timeMusMin;
				msg.m_universeThreadMaxTickTime = timeMusMax;
			}
			msg.m_universeThreadsCount = (uint16_t)universeThreadsTimings.size();

			int64_t timeDiff = universeTime - m_lastSendStatistics;
			assert(timeDiff > 0);
			assert(m_calledGetStateNumAfterLastSendStatistics > 0);
			if (m_lastSendStatistics && timeDiff && m_calledGetStateNumAfterLastSendStatistics)
			{
				msg.m_clientServerPerformanceRatio = ((m_calledGetStateNumAfterLastSendStatistics + m_skippedGetStateAfterLastSendStatistics) * 1000) / timeDiff;
				msg.m_serverClientPerformanceRatio = ((timeDiff * 1000) / m_calledGetStateNumAfterLastSendStatistics);
			}
			// clear client server performance ration
			m_lastSendStatistics = universeTime;
			m_calledGetStateNumAfterLastSendStatistics = 0;
			m_calledGetStateNumAfterLastSendStatistics = 0;
			// send stats
			ParallelPhysics::SendClientMsg(this, msg, sizeof(msg));
		}
		break;
		case MsgType::GetState:
		{
			MsgGetStateResponse msgSendState;
			msgSendState.m_time = universeTime;
			ParallelPhysics::SendClientMsg(this, msgSendState, sizeof(msgSendState));
			++m_calledGetStateNumAfterLastSendStatistics;
			EtherCellPhotonArray photons = ParallelPhysics::GetReceivedPhotons(this);
			for (const Photon &photon : photons)
			{
				if (photon.m_color.m_colorA > 0)
				{
					int8_t posY = photon.m_param / OBSERVER_EYE_SIZE;
					int8_t posX = photon.m_param - posY * OBSERVER_EYE_SIZE;
					MsgSendPhoton msgSendPhoton;
					msgSendPhoton.m_color = photon.m_color;
					msgSendPhoton.m_posX = posX;
					msgSendPhoton.m_posY = posY;
					ParallelPhysics::SendClientMsg(this, msgSendPhoton, sizeof(msgSendPhoton));
				}
			}
		}
		break;
		case MsgType::GetStateExt:
		{
			MsgGetStateExtResponse msgSendState;
			msgSendState.m_latitude = m_latitude;
			msgSendState.m_longitude = m_longitude;
			msgSendState.m_pos = ParallelPhysics::GetObserverPosition(this);
			msgSendState.m_movingProgress = m_movingProgress;
			msgSendState.m_eatenCrumbNum = m_eatenCrumbNum;
			msgSendState.m_eatenCrumbPos = m_eatenCrumbPos;

			ParallelPhysics::SendClientMsg(this, msgSendState, sizeof(msgSendState));
		}
		break;
		case MsgType::MoveForward:
		{
			auto *msg = QueryMessage<MsgMoveForward>(buffer);
			assert(msg);
			MoveForward(msg->m_value);
		}
		break;
		case MsgType::MoveBackward:
		{
			auto *msg = QueryMessage<MsgMoveBackward>(buffer);
			assert(msg);
			MoveBackward(msg->m_value);
		}
		break;
		case MsgType::RotateLeft:
		{
			auto *msg = QueryMessage<MsgRotateLeft>(buffer);
			assert(msg);
			isCalculateEyeStateNeeded |= RotateLeft(msg->m_value);
		}
		break;
		case MsgType::RotateRight:
		{
			auto *msg = QueryMessage<MsgRotateRight>(buffer);
			assert(msg);
			isCalculateEyeStateNeeded |= RotateRight(msg->m_value);
		}
		break;
		case MsgType::RotateUp:
		{
			auto *msg = QueryMessage<MsgRotateUp>(buffer);
			assert(msg);
			isCalculateEyeStateNeeded |= RotateUp(msg->m_value);
		}
		break;
		case MsgType::RotateDown:
		{
			auto *msg = QueryMessage<MsgRotateDown>(buffer);
			assert(msg);
			isCalculateEyeStateNeeded |= RotateDown(msg->m_value);
		}
		break;
		default:
			break;
		}
	}

	if (isCalculateEyeStateNeeded)
	{
		CalculateEyeState();
	}
	Echolocation();
}

void Observer::Echolocation()
{
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		PhotonParam param = yy * OBSERVER_EYE_SIZE + xx;
		ParallelPhysics::EmitEcholocationPhoton(this, m_eyeArray[yy][xx], param);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		PhotonParam param = yy * OBSERVER_EYE_SIZE + xx;
		ParallelPhysics::EmitEcholocationPhoton(this, m_eyeArray[yy][xx], param);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		PhotonParam param = yy * OBSERVER_EYE_SIZE + xx;
		ParallelPhysics::EmitEcholocationPhoton(this, m_eyeArray[yy][xx], param);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		PhotonParam param = yy * OBSERVER_EYE_SIZE + xx;
		ParallelPhysics::EmitEcholocationPhoton(this, m_eyeArray[yy][xx], param);
	}
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
	float len = EYE_FOV / OBSERVER_EYE_SIZE;

	for (int32_t yy = 0; yy < OBSERVER_EYE_SIZE; ++yy)
	{
		for (int32_t xx = 0; xx < OBSERVER_EYE_SIZE; ++xx)
		{
			int16_t latitude = m_latitude + EYE_FOV * yy / OBSERVER_EYE_SIZE - EYE_FOV / 2;
			int16_t longitude = 0;
			int16_t longitudeShift = EYE_FOV * xx / OBSERVER_EYE_SIZE - EYE_FOV / 2;
			if (latitude < -90 || latitude > 90)
			{
				latitude = Sign(latitude) * 180 - latitude;
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

			OrientationVectorMath orient = MaximizePPhOrientation(orientFloat);
			m_eyeArray[yy][xx] = orient;
		}
	}
	CalculateOrientChangers();
}

void Observer::MoveForward(uint8_t value)
{
	auto movingProgressTmp = m_movingProgress;
	m_movingProgress += value;
	if (movingProgressTmp > m_movingProgress)
	{
		m_isMoveForward = true;
	}
}

bool Observer::GrabMoveForward()
{
	bool tmp = m_isMoveForward;
	m_isMoveForward = false;
	return tmp;
}

void Observer::MoveBackward(uint8_t value)
{
	auto movingProgressTmp = m_movingProgress;
	m_movingProgress -= value;
	if (movingProgressTmp < m_movingProgress)
	{
		m_isMoveBackward = true;
	}
}

bool Observer::GrabMoveBackward()
{
	bool tmp = m_isMoveBackward;
	m_isMoveBackward = false;
	return tmp;
}

bool Observer::RotateLeft(uint8_t value)
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
		return true;
	}
	return false;
}

bool Observer::RotateRight(uint8_t value)
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
		return true;
	}
	return false;
}

bool Observer::RotateUp(uint8_t value)
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
			return true;
		}
	}
	return false;
}

bool Observer::RotateDown(uint8_t value)
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
			return true;
		}
	}
	return false;
}

void Observer::IncEatenCrumb(const VectorInt32Math &pos)
{
	m_eatenCrumbPos = pos;
	++m_eatenCrumbNum;
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

	return pphOrientation;
}

void Observer::CalculateOrientChangers()
{
	OrientationVectorMath orientMin(OrientationVectorMath::PPH_INT_MAX, OrientationVectorMath::PPH_INT_MAX, OrientationVectorMath::PPH_INT_MAX);
	OrientationVectorMath orientMax(OrientationVectorMath::PPH_INT_MIN, OrientationVectorMath::PPH_INT_MIN, OrientationVectorMath::PPH_INT_MIN);
	for (int yy = 0; yy < m_eyeArray.size(); ++yy)
	{
		for (int xx = 0; xx < m_eyeArray[yy].size(); ++xx)
		{
			orientMin.m_posX = std::min(orientMin.m_posX, m_eyeArray[yy][xx].m_posX);
			orientMin.m_posY = std::min(orientMin.m_posY, m_eyeArray[yy][xx].m_posY);
			orientMin.m_posZ = std::min(orientMin.m_posZ, m_eyeArray[yy][xx].m_posZ);
			orientMax.m_posX = std::max(orientMax.m_posX, m_eyeArray[yy][xx].m_posX);
			orientMax.m_posY = std::max(orientMax.m_posY, m_eyeArray[yy][xx].m_posY);
			orientMax.m_posZ = std::max(orientMax.m_posZ, m_eyeArray[yy][xx].m_posZ);
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
	ParallelPhysics::SetNeedUpdateSimulationBoxes();
}
} // namespace PPh
