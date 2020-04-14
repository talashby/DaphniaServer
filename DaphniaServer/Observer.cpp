
#include "Observer.h"


namespace PPh
{
Observer::Observer(int32_t index) : m_index(index)
{
	ParallelPhysics::GetInstance()->InitEtherCell(position, EtherType::Observer);
	CalculateEyeState();
}

void Observer::Echolocation()
{
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		Photon photon(m_eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(this, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		Photon photon(m_eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(this, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		Photon photon(m_eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(this, photon);
	}
	{
		int32_t yy = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		int32_t xx = OrientationVectorMath::GetRandomNumber() % (OBSERVER_EYE_SIZE / 2) + (OBSERVER_EYE_SIZE / 2);
		Photon photon(m_eyeArray[yy][xx]);
		photon.m_param = yy * OBSERVER_EYE_SIZE + xx;
		photon.m_color.m_colorA = 255;
		ParallelPhysics::GetInstance()->EmitPhoton(this, photon);
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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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
	static int64_t lastActionTime = 0;
	if (lastActionTime == s_time)
	{
		return; // skip
	}
	lastActionTime = s_time;

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

void Observer::IncEatenCrumb(const VectorInt32Math &pos)
{
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
	ParallelPhysics::GetInstance()->SetNeedUpdateSimulationBoxes();
}
} // namespace PPh
