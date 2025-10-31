#include <Endstops/ZProbe.h>
#include <Platform/RepRap.h>

// ----------------------------------------------------------------------------
// Minimal Z-probe implementation for host builds

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) noexcept
	: EndstopOrZProbe(Z_AXIS),
	  targetAdcValue(DefaultZProbeADValue),
	  number(num),
	  type(ZProbeType::none),
	  sensor(-1),
	  isDeployedByUser(false)
{
	misc.all = 0;
	SetDefaults();
	type = p_type;
}

bool ZProbe::Stopped() const noexcept
{
	return false;
}

EndstopHitDetails ZProbe::CheckTriggered() noexcept
{
	return EndstopHitDetails{};
}

bool ZProbe::Acknowledge(EndstopHitDetails) noexcept
{
	return false;
}

GCodeResult ZProbe::Configure(GCodeBuffer&, const StringRef&, bool&) THROWS(GCodeException)
{
	return GCodeResult::ok;
}

GCodeResult ZProbe::SendProgram(const uint32_t[], size_t, const StringRef&) noexcept
{
	return GCodeResult::ok;
}

GCodeResult ZProbe::HandleG31(GCodeBuffer&, const StringRef&) THROWS(GCodeException)
{
	return GCodeResult::ok;
}

void ZProbe::SetDefaults() noexcept
{
	targetAdcValue = DefaultZProbeADValue;
	for (float& offset : offsets)
	{
		offset = 0.0f;
	}
	offsets[Z_AXIS] = -DefaultZProbeTriggerHeight;

	calibTemperature = DefaultZProbeTemperature;
	for (float& coeff : temperatureCoefficients)
	{
		coeff = 0.0f;
	}

	diveHeights[0] = diveHeights[1] = DefaultZDive;
	const float defaultProbeSpeed = ConvertSpeedFromMmPerSec(DefaultProbingSpeed);
	probeSpeeds[0] = defaultProbeSpeed;
	probeSpeeds[1] = defaultProbeSpeed;
	probeSpeeds[2] = defaultProbeSpeed;
	travelSpeed = ConvertSpeedFromMmPerSec(DefaultZProbeTravelSpeed);
	recoveryTime = 0.0f;
	tolerance = DefaultZProbeTolerance;

	misc.parts.maxTaps = DefaultZProbeTaps;
	misc.parts.turnHeatersOff = false;
	misc.parts.saveToConfigOverride = false;
	misc.parts.probingAway = false;

	type = ZProbeType::none;
	sensor = -1;
	isDeployedByUser = false;
	lastStopHeight = 0.0f;
}

void ZProbe::PrepareForUse(const bool probingAway) noexcept
{
	misc.parts.probingAway = probingAway;
	actualTriggerHeight = -offsets[Z_AXIS] + GetTriggerHeightCompensation();
}

float ZProbe::GetTriggerHeightCompensation() const noexcept
{
	return 0.0f;
}

float ZProbe::GetDiveHeight(int tapsDone) const noexcept
{
	if (FastThenSlowProbing())
	{
		++tapsDone;
	}
	return diveHeights[(tapsDone < 1) ? 0 : 1];
}

float ZProbe::GetStartingHeight(bool firstTap, float previousHeightError) const noexcept
{
	const float baseHeight = (!firstTap && diveHeights[1] < diveHeights[0])
		? diveHeights[1] + previousHeightError
		: diveHeights[0];
	return baseHeight + GetActiveModeTriggerHeight();
}

void ZProbe::SetLastStoppedHeight(float h) noexcept
{
	lastStopHeight = h;
	reprap.SensorsUpdated();
}

int32_t ZProbe::GetReading() const noexcept
{
	return targetAdcValue;
}

int32_t ZProbe::GetSecondaryValues(int32_t& v1) const noexcept
{
	v1 = 0;
	return 0;
}

// ----------------------------------------------------------------------------
// Object model plumbing (host build)

constexpr ObjectModelTableEntry ZProbe::objectModelTable[] = { };
constexpr uint8_t ZProbe::objectModelTableDescriptor[] = { 0 };
constexpr ObjectModelArrayTableEntry ZProbe::objectModelArrayTable[] = { };

DEFINE_GET_OBJECT_MODEL_TABLE(ZProbe)

const ObjectModelArrayTableEntry *_ecv_null ZProbe::GetObjectModelArrayEntry(unsigned int) const noexcept
{
	return nullptr;
}
