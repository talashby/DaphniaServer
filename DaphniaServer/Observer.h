#pragma once

#include "ParallelPhysics.h"

namespace PPh
{
constexpr int8_t EYE_FOV = 90; // Daphnia eye fov
constexpr int32_t OBSERVER_EYE_SIZE = 16; // pixels
constexpr int32_t UPDATE_EYE_TEXTURE_OUT = 20; // milliseconds
typedef std::array< std::array<OrientationVectorMath, OBSERVER_EYE_SIZE>, OBSERVER_EYE_SIZE> EyeArray;
typedef std::array< std::array<EtherColor, OBSERVER_EYE_SIZE>, OBSERVER_EYE_SIZE> EyeColorArray;
typedef std::array< std::array<uint64_t, OBSERVER_EYE_SIZE>, OBSERVER_EYE_SIZE> EyeUpdateTimeArray;
typedef std::shared_ptr< EyeColorArray > SP_EyeColorArray;

class Observer
{
public:
	Observer(int32_t index);

	OrientationVectorMath GetOrientation() const;

	const VectorInt32Math& GetOrientMinChanger() const;
	const VectorInt32Math& GetOrientMaxChanger() const;

	void PPhTick(uint64_t universeTime);
	bool GrabMoveForward();
	bool GrabMoveBackward();
	void IncEatenCrumb(const VectorInt32Math &pos);

	const int32_t m_index;
private:
	void CalculateEyeState();
	void CalculateOrientChangers();
	OrientationVectorMath MaximizePPhOrientation(const VectorFloatMath &orientationVector) const;
	void Echolocation();
	void MoveForward(uint8_t value);
	void MoveBackward(uint8_t value);
	bool RotateLeft(uint8_t value); // returns true if re-CalculateEyeState needed
	bool RotateRight(uint8_t value); // returns true if re-CalculateEyeState needed
	bool RotateUp(uint8_t value); // returns true if re-CalculateEyeState needed
	bool RotateDown(uint8_t value); // returns true if re-CalculateEyeState needed

	const int32_t EYE_IMAGE_DELAY = 3000; // quantum of time
	//const uint32_t EYE_FOV = PPH_INT_MAX/2; // quantum of length (MAX_INT/2 - 90 degrees; MAX_INT - 180 degrees; 2*MAX_INT - 360 degrees)

	const int32_t ECHOLOCATION_FREQUENCY = 1; // quantum of time
	int32_t m_echolocationCounter = 0;
	EyeArray m_eyeArray;

	VectorInt32Math m_orientMinChanger;
	VectorInt32Math m_orientMaxChanger;

	int16_t m_latitude = 0;
	int16_t m_longitude = 0;
	uint16_t m_movingProgress = 0; //
	uint8_t m_latitudeProgress = 0; //
	uint8_t m_longitudeProgress = 0; //

	int16_t m_eatenCrumbNum = 0;
	VectorInt32Math m_eatenCrumbPos = VectorInt32Math::ZeroVector;
	bool m_isMoveForward = false;
	bool m_isMoveBackward = false;
};
} // namespace PPh