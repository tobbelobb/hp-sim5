#pragma once

#ifdef RRF_HOST_BUILD

#include <Movement/Kinematics/Kinematics.h>

class CartesianKinematicsHost : public Kinematics
{
public:
	CartesianKinematicsHost() noexcept;

	MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override;
};

#endif // RRF_HOST_BUILD
