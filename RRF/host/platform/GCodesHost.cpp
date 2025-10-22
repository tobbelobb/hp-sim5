#include <GCodes/GCodes.h>

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodeMachineState.h>
#include <cctype>

GCodes::GCodes() noexcept
{
	UpdateAxisLettersString();
	userPositions.fill(0.0f);
	inputAxesRelative.fill(false);
	inputs.fill(nullptr);
}

void GCodes::RegisterInput(GCodeBuffer& buffer) noexcept
{
	if (numInputs >= inputs.size())
	{
		return;
	}
	inputs[numInputs] = &buffer;
	inputAxesRelative[numInputs] = buffer.LatestMachineState().axesRelative;
	++numInputs;
}

void GCodes::ClearInputs() noexcept
{
	numInputs = 0;
	inputs.fill(nullptr);
	inputAxesRelative.fill(false);
}

bool GCodes::GetAxesRelative(size_t index) const noexcept
{
	return (index < numInputs) ? inputAxesRelative[index] : false;
}

void GCodes::SetAxesRelative(size_t index, bool value) noexcept
{
	if (index >= kMaxInputs)
	{
		return;
	}
	inputAxesRelative[index] = value;
	if (index < numInputs && inputs[index] != nullptr)
	{
		inputs[index]->LatestMachineState().axesRelative = value;
	}
}

void GCodes::SetAxisCount(size_t count) noexcept
{
	const size_t clamped = (count <= kMaxAxes) ? count : kMaxAxes;
	if (clamped != axisCount)
	{
		axisCount = clamped;
		UpdateAxisLettersString();
	}
}

bool GCodes::TryGetAxisIndex(char letter, size_t& index) const noexcept
{
	const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(letter)));
	for (size_t i = 0; i < axisCount; ++i)
	{
		if (axisLetters[i] == upper)
		{
			index = i;
			return true;
		}
	}
	return false;
}

float GCodes::GetUserPosition(size_t axis) const noexcept
{
	return (axis < axisCount) ? userPositions[axis] : 0.0f;
}

void GCodes::SetUserPosition(size_t axis, float value) noexcept
{
	if (axis >= kMaxAxes)
	{
		return;
	}
	if (axis >= axisCount)
	{
		axisCount = axis + 1;
		UpdateAxisLettersString();
	}
	userPositions[axis] = value;
}

void GCodes::ResetUserPositions() noexcept
{
	userPositions.fill(0.0f);
}

void GCodes::UpdateAxisLettersString() noexcept
{
	axisLettersString.fill('\0');
	for (size_t i = 0; i < axisCount && i < kMaxAxes; ++i)
	{
		axisLettersString[i] = axisLetters[i];
	}
}
