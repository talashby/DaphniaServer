#pragma once

#include "PPhHelpers.h"
#include "ServerProtocol.h"
#include "array"

namespace PPh
{
constexpr int8_t EYE_FOV = 120; // Daphnia eye fov
constexpr int32_t OBSERVER_EYE_SIZE_MAX = 16; // pixels
typedef std::array< std::array<OrientationVectorMath, OBSERVER_EYE_SIZE_MAX>, OBSERVER_EYE_SIZE_MAX> EyeArray;

class Observer
{
public:
	Observer(int32_t index, uint8_t eyeSize);

	OrientationVectorMath GetOrientation() const;

	const VectorInt32Math& GetOrientMinChanger() const;
	const VectorInt32Math& GetOrientMaxChanger() const;

	const int16_t& GetLatitude() const;
	const int16_t& GetLongitude() const;

	void PPhTick(uint64_t universeTime);

	bool GrabMoveForward();
	bool GrabMoveBackward();
	bool GrabNewLatitude(); // used to send new orientation to administrator
	bool GrabNewLongitude(); // used to send new orientation to administrator
	bool GetFirstSendToAdmin(); // true - need to send data (position etc.) to admin first time
	void SetFirstSendToAdmin(bool firstSendToAdmin);

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

	bool m_newLatitudeToSend = false;
	bool m_newLongitudeToSend = false;

	int16_t m_eatenCrumbNum = 0;
	VectorInt32Math m_eatenCrumbPos = VectorInt32Math::ZeroVector;
	bool m_isMoveForward = false;
	bool m_isMoveBackward = false;

	uint32_t m_skippedGetStateAfterLastSendStatistics = 0;
	uint32_t m_calledGetStateNumAfterLastSendStatistics = 0;
	uint64_t m_lastSendStatistics = 0;

	bool m_firstSendToAdmin = true;
	const uint8_t m_eyeSize;
};
} // namespace PPh