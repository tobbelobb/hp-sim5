#include "Movement/Kinematics/CartesianKinematicsHost.h"

#ifdef RRF_HOST_BUILD

#include <cmath>

CartesianKinematicsHost::CartesianKinematicsHost() noexcept
	: Kinematics(KinematicsType::cartesian, SegmentationType(false, false, false))
{
}

MovementError CartesianKinematicsHost::CartesianToMotorSteps(const float machinePos[],
															  const float stepsPerMm[],
															  size_t numVisibleAxes,
															  size_t numTotalAxes,
															  int32_t motorPos[],
															  bool /*isCoordinated*/) const noexcept
{
	for (size_t axis = 0; axis < numTotalAxes; ++axis)
	{
		const float pos = (axis < numVisibleAxes) ? machinePos[axis] : machinePos[axis];
		const float spm = stepsPerMm[axis];
		motorPos[axis] = static_cast<int32_t>(std::lround(pos * spm));
	}
	return MovementError::ok;
}

void CartesianKinematicsHost::MotorStepsToCartesian(const int32_t motorPos[],
													const float stepsPerMm[],
													size_t numVisibleAxes,
													size_t numTotalAxes,
													float machinePos[]) const noexcept
{
	for (size_t axis = 0; axis < numTotalAxes; ++axis)
	{
		const float spm = stepsPerMm[axis];
		const float value = static_cast<float>(motorPos[axis]) / ((spm != 0.0f) ? spm : 1.0f);
		if (axis < numVisibleAxes)
		{
			machinePos[axis] = value;
		}
		else
		{
			machinePos[axis] = value;
		}
	}
}

HomingMode CartesianKinematicsHost::GetHomingMode() const noexcept
{
	return HomingMode::homeCartesianAxes;
}

#endif // RRF_HOST_BUILD
