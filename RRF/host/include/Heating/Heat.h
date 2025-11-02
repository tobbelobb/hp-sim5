#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <General/String.h>
#include <General/String.h>

#include <RepRapFirmware.h>
#include <TemperatureError.h>
#include <RTOSIface/RTOSIface.h>

class TemperatureSensor;
class Heater;
class HeaterMonitor;
class GCodeBuffer;
class StringRef;
class Tool;
class FileStore;

struct CanMessageSensorTemperatures;
struct CanMessageHeatersStatus;
struct CanMessageHeaterFeedForwardV1;
struct CanMessageHeaterTuningReport;
struct CanMessageGeneric;
struct CanMessageHeaterModelV2;
struct CanMessageSetHeaterTemperature;
struct CanMessageSetHeaterFaultDetectionParameters;
struct CanMessageSetHeaterMonitors;
struct CanMessageHeaterTuningCommand;

#ifndef RRF_HOST_HEATER_STATUS_DEFINED
#define RRF_HOST_HEATER_STATUS_DEFINED
#include <General/NamedEnum.h>
NamedEnum(HeaterStatus, uint8_t, off, standby, active, fault, tuning, offline);
#endif

class Heat : public ObjectModel
{
public:
	Heat() noexcept;
	Heat(const Heat&) = delete;

	[[noreturn]] void HeaterTask() noexcept;
	void Init() noexcept;
	void Exit() noexcept;
	void ResetHeaterModels() noexcept;

	bool ColdExtrude() const noexcept;
	void AllowColdExtrude(bool b) noexcept;
	float GetExtrusionMinTemp() const noexcept;
	float GetRetractionMinTemp() const noexcept;
	void SetExtrusionMinTemp(float t) noexcept;
	void SetRetractionMinTemp(float t) noexcept;

	int GetBedHeater(size_t index) const noexcept;
	void SetBedHeater(size_t index, int heater) noexcept;
	bool IsBedHeater(int heater) const noexcept;

	int GetChamberHeater(size_t index) const noexcept;
	void SetChamberHeater(size_t index, int heater) noexcept;
	bool IsChamberHeater(int heater) const noexcept;

	void SetAsToolHeater(int8_t heater) const noexcept;
	bool IsBedOrChamberHeater(int heater) const noexcept;

	bool SlowHeatersAtSetTemperatures(float tolerance, bool waitOnFault) const noexcept;

	void SwitchOffAll(bool includingChamberAndBed) noexcept;
	void SwitchOffAllLocalFromISR() noexcept;
	void SuspendHeaters(bool sus) noexcept;
	GCodeResult ResetFault(int heater, const StringRef& reply) noexcept;

	GCodeResult SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult TuneHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ConfigureSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult SetPidParameters(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM143(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException);

	ReadLockedPointer<TemperatureSensor> FindSensor(int sensorNumber) const noexcept;
	ReadLockedPointer<TemperatureSensor> FindSensorAtOrAbove(unsigned int sensorNumber) const noexcept;

	float GetSensorTemperature(int sensorNum, TemperatureError& err) const noexcept;

	float GetHighestTemperatureLimit() const noexcept;
	size_t GetNumHeatersToReport() const noexcept;
	size_t GetNumSensorsToReport() const noexcept;

	void Diagnostics(const StringRef& reply) noexcept;

	float GetAveragePWM(size_t heater) const noexcept;
	const Tool *_ecv_null GetLastStandbyTool(int heater) const noexcept;
	bool IsHeaterEnabled(size_t heater) const noexcept;

	float GetActiveTemperature(int heater) const noexcept;
	float GetStandbyTemperature(int heater) const noexcept;
	float GetHighestTemperatureLimit(int heater) const noexcept;
	float GetLowestTemperatureLimit(int heater) const noexcept;
	float GetTargetTemperature(int heater) const noexcept;
	float GetHeaterTemperature(int heater) const noexcept;
	HeaterStatus GetStatus(int heater) const noexcept;
	bool HeaterAtSetTemperature(int heater, bool waitWhenCooling, float tolerance, bool waitOnFault) const noexcept;

	GCodeResult ConfigureHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ConfigureHeaterMonitoring(size_t heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	void SetActiveTemperature(int heater, float t) THROWS(GCodeException) { SetTemperature(heater, t, true); }
	void SetStandbyTemperature(int heater, float t) THROWS(GCodeException) { SetTemperature(heater, t, false); }
	void SetTemperature(int heater, float t, bool activeNotStandby) THROWS(GCodeException);

	GCodeResult SetActiveOrStandby(int heater, const Tool *_ecv_null tool, bool active, const StringRef& reply) noexcept;
	void SwitchOff(int heater) noexcept;
	void SetFanFeedForwardPwm(unsigned int heater, float fanPwm) const noexcept;
	void SetExtrusionFeedForward(unsigned int heater, float pwmBoost, float tempBoost) const noexcept;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteModelParameters(FileStore *f) const noexcept;
	bool WriteBedAndChamberTempSettings(FileStore *f) const noexcept;
#endif

#if SUPPORT_CAN_EXPANSION
	void ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept;
	void ProcessRemoteHeatersReport(CanAddress src, const CanMessageHeatersStatus& msg) noexcept;
	void ProcessRemoteHeaterTuningReport(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept;
#endif

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult ConfigureHeater(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM307V1(const CanMessageHeaterModelV2& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM308(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;
	GCodeResult SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply) noexcept;
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept;
	GCodeResult ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept;
#endif

	static TaskHandle GetHeatTask() noexcept { return heatTaskHandle; }

	static inline ReadWriteLock sensorsLock{};
	static inline ReadWriteLock heatersLock{};

protected:
	const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept override { return nullptr; }
	const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int) const noexcept override { return nullptr; }

private:
	struct SensorState
	{
		bool configured{false};
		String<StringLength50> portName{};
		String<StringLength50> typeName{};
		float lastTemperature{AmbientTemperature};
	};

	struct HeaterConfig
	{
		bool created{false};
		String<StringLength50> portName{};
		int sensorNumber{-1};
		float pwmFrequency{static_cast<float>(DefaultHeaterPwmFreq)};
		bool usePid{true};
		float pwmLimit{1.0f};
		float modelGain{0.0f};
		float timeConstant{0.0f};
		float deadTime{0.0f};
		float supplyVoltage{0.0f};
		float lowTemperatureLimit{NEARLY_ABS_ZERO};
		float highTemperatureLimit{500.0f};
		float faultTimeout{0.0f};
	};

	struct HeaterState
	{
		bool configured{false};
		bool enabled{false};
		bool suspended{false};
		HeaterStatus status{HeaterStatus::off};
		float currentTemperature{AmbientTemperature};
		float activeTemperature{0.0f};
		float standbyTemperature{0.0f};
		float minTemperatureLimit{NEARLY_ABS_ZERO};
		float maxTemperatureLimit{500.0f};
		float averagePwm{0.0f};
		const Tool *_ecv_null lastStandbyTool{nullptr};
	};

	static constexpr float AmbientTemperature = 25.0f;

	static inline TaskHandle heatTaskHandle{nullptr};

	bool IsValidHeater(int heater) const noexcept;
	HeaterState& GetState(int heater) noexcept;
	const HeaterState& GetState(int heater) const noexcept;
	template <size_t N>
	bool HeaterMatchesList(int heater, const std::array<int8_t, N>& list) const noexcept;

	SensorState& GetSensorState(size_t sensor) noexcept { return sensorStates[sensor]; }
	const SensorState& GetSensorState(size_t sensor) const noexcept { return sensorStates[sensor]; }
	HeaterConfig& GetHeaterConfig(size_t heater) noexcept { return heaterConfigs[heater]; }
	const HeaterConfig& GetHeaterConfig(size_t heater) const noexcept { return heaterConfigs[heater]; }

	std::array<SensorState, MaxSensors> sensorStates{};
	std::array<HeaterConfig, MaxHeaters> heaterConfigs{};
	std::array<HeaterState, MaxHeaters> heaterStates{};
	std::array<int8_t, MaxBedHeaters> bedHeaters{};
	std::array<int8_t, MaxChamberHeaters> chamberHeaters{};

	float extrusionMinTemp{0.0f};
	float retractionMinTemp{0.0f};
	bool coldExtrude{false};
	bool heatersSuspended{false};
};

inline Heat::Heat() noexcept
{
	bedHeaters.fill(-1);
	chamberHeaters.fill(-1);
	for (auto& sensor : sensorStates)
	{
		sensor.lastTemperature = AmbientTemperature;
	}
	for (auto& config : heaterConfigs)
	{
		config.lowTemperatureLimit = NEARLY_ABS_ZERO;
		config.highTemperatureLimit = 500.0f;
	}
}

[[noreturn]] inline void Heat::HeaterTask() noexcept
{
	std::abort();
}

inline void Heat::Init() noexcept
{
	heatersSuspended = false;
}

inline void Heat::Exit() noexcept
{
	SwitchOffAll(true);
}

inline void Heat::ResetHeaterModels() noexcept
{
	// Nothing to do for host stub
}

inline bool Heat::ColdExtrude() const noexcept
{
	return coldExtrude;
}

inline void Heat::AllowColdExtrude(bool b) noexcept
{
	coldExtrude = b;
}

inline float Heat::GetExtrusionMinTemp() const noexcept
{
	return extrusionMinTemp;
}

inline float Heat::GetRetractionMinTemp() const noexcept
{
	return retractionMinTemp;
}

inline void Heat::SetExtrusionMinTemp(float t) noexcept
{
	extrusionMinTemp = t;
}

inline void Heat::SetRetractionMinTemp(float t) noexcept
{
	retractionMinTemp = t;
}

inline bool Heat::IsValidHeater(int heater) const noexcept
{
	return heater >= 0 && static_cast<size_t>(heater) < heaterStates.size();
}

inline Heat::HeaterState& Heat::GetState(int heater) noexcept
{
	return heaterStates[static_cast<size_t>(heater)];
}

inline const Heat::HeaterState& Heat::GetState(int heater) const noexcept
{
	return heaterStates[static_cast<size_t>(heater)];
}

template <size_t N>
inline bool Heat::HeaterMatchesList(int heater, const std::array<int8_t, N>& list) const noexcept
{
	if (!IsValidHeater(heater))
	{
		return false;
	}
	for (int8_t entry : list)
	{
		if (entry == heater)
		{
			return true;
		}
	}
	return false;
}

inline int Heat::GetBedHeater(size_t index) const noexcept
{
	return (index < bedHeaters.size()) ? bedHeaters[index] : -1;
}

inline void Heat::SetBedHeater(size_t index, int heater) noexcept
{
	if (index < bedHeaters.size())
	{
		bedHeaters[index] = (IsValidHeater(heater)) ? static_cast<int8_t>(heater) : -1;
	}
}

inline bool Heat::IsBedHeater(int heater) const noexcept
{
	return HeaterMatchesList(heater, bedHeaters);
}

inline int Heat::GetChamberHeater(size_t index) const noexcept
{
	return (index < chamberHeaters.size()) ? chamberHeaters[index] : -1;
}

inline void Heat::SetChamberHeater(size_t index, int heater) noexcept
{
	if (index < chamberHeaters.size())
	{
		chamberHeaters[index] = (IsValidHeater(heater)) ? static_cast<int8_t>(heater) : -1;
	}
}

inline bool Heat::IsChamberHeater(int heater) const noexcept
{
	return HeaterMatchesList(heater, chamberHeaters);
}

inline void Heat::SetAsToolHeater(int8_t) const noexcept
{
	// Host build does not distinguish tool heaters
}

inline bool Heat::IsBedOrChamberHeater(int heater) const noexcept
{
	return IsBedHeater(heater) || IsChamberHeater(heater);
}

inline bool Heat::SlowHeatersAtSetTemperatures(float, bool) const noexcept
{
	// Host stub assumes heaters are always at target temperature
	return true;
}

inline void Heat::SwitchOffAll(bool) noexcept
{
	for (int heater = 0; heater < static_cast<int>(heaterStates.size()); ++heater)
	{
		SwitchOff(heater);
	}
}

inline void Heat::SwitchOffAllLocalFromISR() noexcept
{
	SwitchOffAll(true);
}

inline void Heat::SuspendHeaters(bool sus) noexcept
{
	heatersSuspended = sus;
}

inline GCodeResult Heat::ResetFault(int heater, const StringRef&) noexcept
{
	if (IsValidHeater(heater))
	{
		auto& state = GetState(heater);
		state.status = HeaterStatus::off;
		state.enabled = false;
		state.averagePwm = 0.0f;
		state.currentTemperature = AmbientTemperature;
	}
	return GCodeResult::ok;
}

inline GCodeResult Heat::SetOrReportHeaterModel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int heater = gb.GetLimitedUIValue('H', MaxHeaters);
	auto& config = GetHeaterConfig(heater);
	auto& state = GetState(static_cast<int>(heater));
	bool seenParameter = false;

	if (gb.Seen('B'))
	{
		config.usePid = (gb.GetIValue() == 0);
		seenParameter = true;
	}

	bool seen = false;
	float value = 0.0f;
	if (gb.TryGetFValue('S', value, seen) && seen)
	{
		config.pwmLimit = std::clamp(value, 0.0f, 1.0f);
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('A', value, seen) && seen)
	{
		config.modelGain = value;
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('C', value, seen) && seen)
	{
		config.timeConstant = value;
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('D', value, seen) && seen)
	{
		config.deadTime = value;
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('V', value, seen) && seen)
	{
		config.supplyVoltage = value;
		seenParameter = true;
	}

	if (seenParameter)
	{
		config.created = true;
		state.configured = true;
	}
	else
	{
		reply.printf(
			"Heater %u model: %s, S=%.2f, A=%.2f, C=%.2f, D=%.2f, V=%.1f",
			heater,
			config.usePid ? "PID" : "bang-bang",
			(double)config.pwmLimit,
			(double)config.modelGain,
			(double)config.timeConstant,
			(double)config.deadTime,
			(double)config.supplyVoltage);
	}

	return GCodeResult::ok;
}

inline GCodeResult Heat::TuneHeater(GCodeBuffer&, const StringRef&) THROWS(GCodeException)
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::ConfigureSensor(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	gb.MustSee('S');
	const unsigned sensorNum = gb.GetUIValue();
	if (sensorNum >= MaxSensors)
	{
		reply.copy("Sensor number out of range");
		return GCodeResult::error;
	}

	auto& sensor = GetSensorState(sensorNum);
	bool anyParameter = false;
	bool changed = false;

	if (gb.Seen('P'))
	{
		String<StringLength50> port;
		gb.GetReducedString(port.GetRef());
		if (port.EqualsIgnoreCase(NoPinName))
		{
			sensor = SensorState{};
			sensor.lastTemperature = AmbientTemperature;
			return GCodeResult::ok;
		}

		sensor.portName.copy(port.c_str());
		sensor.configured = true;
		anyParameter = true;
		changed = true;
	}

	if (gb.Seen('Y'))
	{
		String<StringLength50> type;
		gb.GetReducedString(type.GetRef());
		sensor.typeName.copy(type.c_str());
		sensor.configured = true;
		anyParameter = true;
		changed = true;
	}

	bool seen = false;
	float value = 0.0f;
	if (gb.TryGetFValue('T', value, seen) && seen)
	{
		anyParameter = true;
		changed = true;
	}
	seen = false;
	if (gb.TryGetFValue('B', value, seen) && seen)
	{
		anyParameter = true;
		changed = true;
	}
	seen = false;
	if (gb.TryGetFValue('C', value, seen) && seen)
	{
		anyParameter = true;
		changed = true;
	}
	seen = false;
	if (gb.TryGetFValue('R', value, seen) && seen)
	{
		anyParameter = true;
		changed = true;
	}

	if (changed && !sensor.configured)
	{
		sensor.configured = true;
	}

	if (!anyParameter)
	{
		if (!sensor.configured)
		{
			reply.printf("Sensor %u not configured", sensorNum);
		}
		else
		{
			reply.printf(
			"Sensor %u type %s reading %.1fC",
			sensorNum,
			(sensor.typeName.IsEmpty()) ? "unknown" : sensor.typeName.c_str(),
			(double)sensor.lastTemperature);
		}
	}

	return GCodeResult::ok;
}

inline GCodeResult Heat::SetPidParameters(unsigned int, GCodeBuffer&, const StringRef&) THROWS(GCodeException)
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::HandleM143(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t heaterNumber = (gb.Seen('H')) ? gb.GetLimitedUIValue('H', MaxHeaters) : 1;
	if (!IsValidHeater(static_cast<int>(heaterNumber)))
	{
		reply.printf("Heater %zu does not exist", heaterNumber);
		return GCodeResult::error;
	}

	auto& config = GetHeaterConfig(heaterNumber);
	auto& state = GetState(static_cast<int>(heaterNumber));
	bool seenParameter = false;

	bool seen = false;
	float value = 0.0f;
	if (gb.TryGetFValue('S', value, seen) && seen)
	{
		config.highTemperatureLimit = value;
		state.maxTemperatureLimit = value;
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('L', value, seen) && seen)
	{
		config.lowTemperatureLimit = value;
		state.minTemperatureLimit = value;
		seenParameter = true;
	}

		if (!seenParameter)
		{
			reply.printf(
			"Heater %zu limits %.1f to %.1fC",
			heaterNumber,
			(double)state.minTemperatureLimit,
			(double)state.maxTemperatureLimit);
	}
	else
	{
		config.created = true;
		state.configured = true;
	}

	return GCodeResult::ok;
}

inline ReadLockedPointer<TemperatureSensor> Heat::FindSensor(int) const noexcept
{
	return ReadLockedPointer<TemperatureSensor>(nullptr, nullptr);
}

inline ReadLockedPointer<TemperatureSensor> Heat::FindSensorAtOrAbove(unsigned int) const noexcept
{
	return ReadLockedPointer<TemperatureSensor>(nullptr, nullptr);
}

inline float Heat::GetSensorTemperature(int sensorNum, TemperatureError& err) const noexcept
{
	if (sensorNum >= 0 && static_cast<size_t>(sensorNum) < sensorStates.size())
	{
		const auto& sensor = GetSensorState(static_cast<size_t>(sensorNum));
		if (sensor.configured)
		{
			err = TemperatureError::ok;
			return sensor.lastTemperature;
		}
	}

	err = TemperatureError::unknownSensor;
	return AmbientTemperature;
}

inline float Heat::GetHighestTemperatureLimit() const noexcept
{
	float limit = 0.0f;
	for (const auto& state : heaterStates)
	{
		limit = std::max(limit, state.maxTemperatureLimit);
	}
	return limit;
}

inline size_t Heat::GetNumHeatersToReport() const noexcept
{
	size_t count = 0;
	for (const auto& state : heaterStates)
	{
		if (state.configured)
		{
			++count;
		}
	}
	return count;
}

inline size_t Heat::GetNumSensorsToReport() const noexcept
{
	return 0;
}

inline void Heat::Diagnostics(const StringRef&) noexcept
{
	// Nothing to report in host mode
}

inline float Heat::GetAveragePWM(size_t heater) const noexcept
{
	return (heater < heaterStates.size()) ? heaterStates[heater].averagePwm : 0.0f;
}

inline const Tool *_ecv_null Heat::GetLastStandbyTool(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).lastStandbyTool : nullptr;
}

inline bool Heat::IsHeaterEnabled(size_t heater) const noexcept
{
	return (heater < heaterStates.size()) && heaterStates[heater].enabled;
}

inline float Heat::GetActiveTemperature(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).activeTemperature : 0.0f;
}

inline float Heat::GetStandbyTemperature(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).standbyTemperature : 0.0f;
}

inline float Heat::GetHighestTemperatureLimit(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).maxTemperatureLimit : 0.0f;
}

inline float Heat::GetLowestTemperatureLimit(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).minTemperatureLimit : 0.0f;
}

inline float Heat::GetTargetTemperature(int heater) const noexcept
{
	if (!IsValidHeater(heater))
	{
		return 0.0f;
	}
	const auto& state = GetState(heater);
	if (state.status == HeaterStatus::active)
	{
		return state.activeTemperature;
	}
	if (state.status == HeaterStatus::standby)
	{
		return state.standbyTemperature;
	}
	return 0.0f;
}

inline float Heat::GetHeaterTemperature(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).currentTemperature : 0.0f;
}

inline HeaterStatus Heat::GetStatus(int heater) const noexcept
{
	return (IsValidHeater(heater)) ? GetState(heater).status : HeaterStatus::offline;
}

inline bool Heat::HeaterAtSetTemperature(int heater, bool, float tolerance, bool) const noexcept
{
	if (!IsValidHeater(heater))
	{
		return false;
	}
	const auto& state = GetState(heater);
	const float target = GetTargetTemperature(heater);
	return std::fabs(state.currentTemperature - target) <= tolerance;
}

inline GCodeResult Heat::ConfigureHeater(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t heater = gb.GetLimitedUIValue('H', MaxHeaters);
	auto& config = GetHeaterConfig(heater);
	auto& state = GetState(static_cast<int>(heater));
	bool seenParameter = false;

	if (gb.Seen('C'))
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());
		if (pinName.EqualsIgnoreCase(NoPinName))
		{
			SwitchOff(static_cast<int>(heater));
			config = HeaterConfig{};
			config.lowTemperatureLimit = NEARLY_ABS_ZERO;
			config.highTemperatureLimit = 500.0f;
			state = HeaterState{};
			state.minTemperatureLimit = NEARLY_ABS_ZERO;
			state.maxTemperatureLimit = 500.0f;
			reply.printf("Heater %zu deleted", heater);
			return GCodeResult::ok;
		}

		config.portName.copy(pinName.c_str());
		config.created = true;
		state.configured = true;
		seenParameter = true;
	}

	if (gb.Seen('T'))
	{
		const unsigned sensorNum = gb.GetUIValue();
		if (sensorNum >= MaxSensors || !GetSensorState(sensorNum).configured)
		{
			reply.printf("Sensor %u not configured", sensorNum);
			return GCodeResult::error;
		}
		config.sensorNumber = static_cast<int>(sensorNum);
		config.created = true;
		state.configured = true;
		seenParameter = true;
	}

	if (gb.Seen('Q'))
	{
		config.pwmFrequency = static_cast<float>(gb.GetPwmFrequency());
		config.created = true;
		state.configured = true;
		seenParameter = true;
	}

	state.minTemperatureLimit = config.lowTemperatureLimit;
	state.maxTemperatureLimit = config.highTemperatureLimit;

	if (!seenParameter)
	{
		if (!config.created)
		{
			reply.printf("Heater %zu not configured", heater);
		}
		else
		{
			reply.printf(
			"Heater %zu: port %s, sensor %d, freq %.1f Hz",
			heater,
			config.portName.IsEmpty() ? "unset" : config.portName.c_str(),
			config.sensorNumber,
			(double)config.pwmFrequency);
		}
	}

	return GCodeResult::ok;
}

inline GCodeResult Heat::ConfigureHeaterMonitoring(size_t heater, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (heater >= MaxHeaters)
	{
		reply.copy("Heater number out of range");
		return GCodeResult::error;
	}

	auto& config = GetHeaterConfig(heater);
	bool seenParameter = false;
	bool seen = false;
	float value = 0.0f;
	if (gb.TryGetFValue('S', value, seen) && seen)
	{
		config.faultTimeout = value;
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('P', value, seen) && seen)
	{
		seenParameter = true;
	}
	seen = false;
	if (gb.TryGetFValue('T', value, seen) && seen)
	{
		seenParameter = true;
	}

	if (!seenParameter)
	{
		reply.printf("Heater %zu fault timeout %.1f s", heater, (double)config.faultTimeout);
	}
	else
	{
		config.created = true;
	}

	return GCodeResult::ok;
}

inline void Heat::SetTemperature(int heater, float t, bool activeNotStandby) THROWS(GCodeException)
{
	if (!IsValidHeater(heater))
	{
		return;
	}

	auto& state = GetState(heater);
	const auto& config = GetHeaterConfig(static_cast<size_t>(heater));
	state.configured = true;
	state.enabled = (t > 0.0f);
	state.averagePwm = state.enabled ? 1.0f : 0.0f;

	if (activeNotStandby)
	{
		state.activeTemperature = t;
		state.status = state.enabled ? HeaterStatus::active : HeaterStatus::off;
	}
	else
	{
		state.standbyTemperature = t;
		state.status = state.enabled ? HeaterStatus::standby : HeaterStatus::off;
	}

	if (state.status == HeaterStatus::active)
	{
		state.currentTemperature = state.activeTemperature;
	}
	else if (state.status == HeaterStatus::standby)
	{
		state.currentTemperature = state.standbyTemperature;
	}
	else
	{
		state.currentTemperature = 0.0f;
	}

	if (config.sensorNumber >= 0 && static_cast<size_t>(config.sensorNumber) < sensorStates.size())
	{
		auto& sensor = GetSensorState(static_cast<size_t>(config.sensorNumber));
		if (sensor.configured)
		{
			sensor.lastTemperature = state.currentTemperature;
		}
	}
}

inline GCodeResult Heat::SetActiveOrStandby(int heater, const Tool *_ecv_null tool, bool active, const StringRef&) noexcept
{
	if (!IsValidHeater(heater))
	{
		return GCodeResult::error;
	}

	auto& state = GetState(heater);
	state.configured = true;
	state.enabled = active || state.standbyTemperature > 0.0f;
	state.status = active ? HeaterStatus::active : HeaterStatus::standby;
	state.averagePwm = state.enabled ? 1.0f : 0.0f;
	if (state.status == HeaterStatus::active)
	{
		state.currentTemperature = state.activeTemperature;
	}
	else
	{
		state.currentTemperature = state.standbyTemperature;
		state.lastStandbyTool = tool;
	}

	const auto& config = GetHeaterConfig(static_cast<size_t>(heater));
	if (config.sensorNumber >= 0 && static_cast<size_t>(config.sensorNumber) < sensorStates.size())
	{
		auto& sensor = GetSensorState(static_cast<size_t>(config.sensorNumber));
		if (sensor.configured)
		{
			sensor.lastTemperature = state.currentTemperature;
		}
	}

	return GCodeResult::ok;
}

inline void Heat::SwitchOff(int heater) noexcept
{
	if (!IsValidHeater(heater))
	{
		return;
	}
	auto& state = GetState(heater);
	state.enabled = false;
	state.status = HeaterStatus::off;
	state.currentTemperature = AmbientTemperature;
	state.averagePwm = 0.0f;
	const auto& config = GetHeaterConfig(static_cast<size_t>(heater));
	if (config.sensorNumber >= 0 && static_cast<size_t>(config.sensorNumber) < sensorStates.size())
	{
		auto& sensor = GetSensorState(static_cast<size_t>(config.sensorNumber));
		if (sensor.configured)
		{
			sensor.lastTemperature = AmbientTemperature;
		}
	}
}

inline void Heat::SetFanFeedForwardPwm(unsigned int, float) const noexcept
{
	// Host stub ignores fan feed-forward
}

inline void Heat::SetExtrusionFeedForward(unsigned int, float, float) const noexcept
{
	// Host stub ignores extrusion feed-forward
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
inline bool Heat::WriteModelParameters(FileStore *) const noexcept
{
	return true;
}

inline bool Heat::WriteBedAndChamberTempSettings(FileStore *) const noexcept
{
	return true;
}
#endif

#if SUPPORT_CAN_EXPANSION
inline void Heat::ProcessRemoteSensorsReport(CanAddress, const CanMessageSensorTemperatures&) noexcept
{
}

inline void Heat::ProcessRemoteHeatersReport(CanAddress, const CanMessageHeatersStatus&) noexcept
{
}

inline void Heat::ProcessRemoteHeaterTuningReport(CanAddress, const CanMessageHeaterTuningReport&) noexcept
{
}
#endif

#if SUPPORT_REMOTE_COMMANDS
inline GCodeResult Heat::ConfigureHeater(const CanMessageGeneric&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::ProcessM307V1(const CanMessageHeaterModelV2&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::ProcessM308(const CanMessageGeneric&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::SetHeaterMonitors(const CanMessageSetHeaterMonitors&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::SetTemperature(const CanMessageSetHeaterTemperature&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::TuningCommand(const CanMessageHeaterTuningCommand&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}

inline GCodeResult Heat::ApplyFeedForward(const CanMessageHeaterFeedForwardV1&, const StringRef&) noexcept
{
	return GCodeResult::warningNotSupported;
}
#endif
