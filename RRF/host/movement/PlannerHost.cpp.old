#include <Movement/Planner.h> // Should be shadowed...

#ifdef RRF_HOST_BUILD

#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <CAN/CanMotion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace host::planner
{
	namespace
	{
		constexpr double kTiny = 1e-9;

		struct QueuedSegment
		{
			RawMove move{};
			std::array<float, MaxAxesPlusExtruders> startCoords{};
		};

		struct SegmentPlan
		{
			RawMove move{};
			std::array<float, MaxAxesPlusExtruders> startCoords{};
			std::array<double, MaxAxesPlusExtruders> unitDir{};
			double travelDistance{0.0};
			double accelLimit{1.0};
			double maxSpeed{1.0};
			double entrySpeed{0.0};
			double exitSpeed{0.0};
			double topSpeed{0.0};
			double extruderDelta{0.0};
		};

		inline std::vector<QueuedSegment>& segment_queue() noexcept
		{
			static std::vector<QueuedSegment> queue;
			return queue;
		}

		inline uint64_t& accumulated_ticks() noexcept
		{
			static uint64_t ticks = 0;
			return ticks;
		}

		inline bool axis_in_mask(const RawMove& mv, size_t axis) noexcept
		{
			if ((mv.flags & RMF_RawMotorMove) == 0)
			{
				return true;
			}
			return (mv.independentMask & (1u << axis)) != 0;
		}
	} // namespace

	void QueueSegment(const RawMove& move, const float startMachineCoords[MaxAxesPlusExtruders]) noexcept
	{
		QueuedSegment seg;
		seg.move = move;
		for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
		{
			seg.startCoords[i] = startMachineCoords[i];
		}
		segment_queue().push_back(seg);
	}

	void Reset() noexcept
	{
		segment_queue().clear();
		accumulated_ticks() = 0;
	}

	bool FlushQueuedSegments() noexcept
	{
		auto& queue = segment_queue();
		if (queue.empty())
		{
			return true;
		}

		Move& move = reprap.GetMove();
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();

		std::vector<SegmentPlan> plans;
		plans.reserve(queue.size());

		for (const QueuedSegment& seg : queue)
		{
			SegmentPlan plan;
			plan.move = seg.move;
			plan.startCoords = seg.startCoords;
			plan.unitDir.fill(0.0);

			std::array<double, MaxAxesPlusExtruders> deltas{};
			double sumSq = 0.0;
			for (size_t axis = 0; axis < visibleAxes && axis < MaxAxes; ++axis)
			{
				if (!axis_in_mask(plan.move, axis))
				{
					continue;
				}
				const double delta = static_cast<double>(plan.move.coords[axis] - plan.startCoords[axis]);
				deltas[axis] = delta;
				sumSq += delta * delta;
			}

			const double axisDistance = std::sqrt(sumSq);
			const double extruderDelta = static_cast<double>(plan.move.coords[MaxAxes] - plan.startCoords[MaxAxes]);
			deltas[MaxAxes] = extruderDelta;
			plan.extruderDelta = extruderDelta;

			if (axisDistance > kTiny)
			{
				const double invLength = 1.0 / axisDistance;
				for (size_t axis = 0; axis < MaxAxes; ++axis)
				{
					plan.unitDir[axis] = deltas[axis] * invLength;
				}
				plan.unitDir[MaxAxes] = extruderDelta * invLength;
				plan.travelDistance = axisDistance;
			}
			else if (std::abs(extruderDelta) > kTiny)
			{
				plan.unitDir[MaxAxes] = (extruderDelta > 0.0) ? 1.0 : -1.0;
				plan.travelDistance = std::abs(extruderDelta);
			}
			else
			{
				plan.travelDistance = 0.0;
			}

			double accelLimit = std::numeric_limits<double>::max();
			double speedLimit = std::numeric_limits<double>::max();

			for (size_t axis = 0; axis < visibleAxes && axis < MaxAxes; ++axis)
			{
				const double dir = std::abs(plan.unitDir[axis]);
				if (dir < kTiny)
				{
					continue;
				}
				const double axisAccel = static_cast<double>(move.GetAcceleration(axis));
				const double axisSpeed = static_cast<double>(move.GetMaxFeedrate(axis));
				accelLimit = std::min(accelLimit, axisAccel / dir);
				speedLimit = std::min(speedLimit, axisSpeed / dir);
			}

			if (std::abs(plan.unitDir[MaxAxes]) > kTiny)
			{
				const double dir = std::abs(plan.unitDir[MaxAxes]);
				const size_t extruderDrive = MaxAxes;
				const double extruderAccel = static_cast<double>(move.GetAcceleration(extruderDrive));
				const double extruderSpeed = static_cast<double>(move.GetMaxFeedrate(extruderDrive));
				accelLimit = std::min(accelLimit, extruderAccel / dir);
				speedLimit = std::min(speedLimit, extruderSpeed / dir);
			}

			if (accelLimit == std::numeric_limits<double>::max())
			{
				accelLimit = static_cast<double>(move.GetAcceleration(0));
			}

			double requestedSpeed = static_cast<double>(plan.move.feedRate);
			if (requestedSpeed <= 0.0)
			{
				requestedSpeed = static_cast<double>(move.GetMaxFeedrate(0));
			}

			double maxSpeed = std::min(requestedSpeed, speedLimit);
			if (!std::isfinite(maxSpeed) || maxSpeed <= 0.0)
			{
				maxSpeed = requestedSpeed;
			}

			maxSpeed = std::max(maxSpeed, 1e-3);
			accelLimit = std::max(accelLimit, 1.0);

			plan.accelLimit = accelLimit;
			plan.maxSpeed = maxSpeed;
			plan.entrySpeed = 0.0;
			plan.exitSpeed = maxSpeed;
			plan.topSpeed = maxSpeed;

			plans.push_back(plan);
		}

		if (plans.empty())
		{
			queue.clear();
			return true;
		}

		std::vector<double> junctionLimit;
		if (plans.size() > 1)
		{
			junctionLimit.resize(plans.size() - 1, std::numeric_limits<double>::infinity());
			for (size_t i = 0; i + 1 < plans.size(); ++i)
			{
				double limit = std::numeric_limits<double>::infinity();
				for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
				{
					const double diff = std::abs(plans[i].unitDir[drive] - plans[i + 1].unitDir[drive]);
					if (diff < kTiny)
					{
						continue;
					}

					const double allowed = static_cast<double>(move.GetPrintingInstantDv(drive));
					if (allowed <= 0.0)
					{
						limit = 0.0;
						break;
					}
					limit = std::min(limit, allowed / diff);
				}
				if (!std::isfinite(limit))
				{
					limit = std::min(plans[i].maxSpeed, plans[i + 1].maxSpeed);
				}
				limit = std::max(0.0, limit);
				limit = std::min(limit, std::min(plans[i].maxSpeed, plans[i + 1].maxSpeed));
				junctionLimit[i] = limit;
			}
		}

		std::vector<double> exitSpeeds(plans.size());
		for (size_t i = 0; i < plans.size(); ++i)
		{
			exitSpeeds[i] = plans[i].maxSpeed;
		}

		const double epsilon = 1e-6;
		bool changed = true;
		size_t iteration = 0;
		const size_t maxIterations = 64;
		while (changed && iteration < maxIterations)
		{
			changed = false;
			double entry = 0.0;
			for (size_t i = 0; i < plans.size(); ++i)
			{
				plans[i].entrySpeed = entry;
				const double reachable = std::sqrt(entry * entry + 2.0 * plans[i].accelLimit * std::max(plans[i].travelDistance, kTiny));
				double limit = plans[i].maxSpeed;
				if (i < junctionLimit.size())
				{
					limit = std::min(limit, junctionLimit[i]);
				}
				double newExit = std::min(exitSpeeds[i], reachable);
				newExit = std::min(newExit, limit);
				newExit = std::max(0.0, newExit);
				if (exitSpeeds[i] - newExit > epsilon)
				{
					exitSpeeds[i] = newExit;
					changed = true;
				}
				else
				{
					exitSpeeds[i] = newExit;
				}
				entry = exitSpeeds[i];
			}

			double nextSpeed = 0.0;
			for (size_t idx = plans.size(); idx-- > 0;)
			{
				double limit = (idx < junctionLimit.size()) ? junctionLimit[idx] : 0.0;
				double newExit = exitSpeeds[idx];
				if (idx == plans.size() - 1)
				{
					if (newExit > epsilon)
					{
						newExit = 0.0;
						changed = true;
					}
				}
				else if (newExit - limit > epsilon)
				{
					newExit = limit;
					changed = true;
				}

				const double reachable = std::sqrt(nextSpeed * nextSpeed + 2.0 * plans[idx].accelLimit * std::max(plans[idx].travelDistance, kTiny));
				if (newExit - reachable > epsilon)
				{
					newExit = reachable;
					changed = true;
				}
				newExit = std::max(0.0, newExit);

				if (exitSpeeds[idx] - newExit > epsilon)
				{
					exitSpeeds[idx] = newExit;
					changed = true;
				}
				else
				{
					exitSpeeds[idx] = newExit;
				}
				nextSpeed = exitSpeeds[idx];
			}

			++iteration;
		}

		double prevExit = 0.0;
		for (size_t i = 0; i < plans.size(); ++i)
		{
			plans[i].entrySpeed = prevExit;
			plans[i].exitSpeed = std::max(0.0, exitSpeeds[i]);
			prevExit = plans[i].exitSpeed;

			const double accel = plans[i].accelLimit;
			const double length = std::max(plans[i].travelDistance, kTiny);
			const double vEntry = plans[i].entrySpeed;
			const double vExit = plans[i].exitSpeed;
			double vPeak = plans[i].maxSpeed;

			double accelDistance = (vPeak > vEntry)
									 ? (vPeak * vPeak - vEntry * vEntry) / (2.0 * accel)
									 : 0.0;
			double decelDistance = (vPeak > vExit)
									 ? (vPeak * vPeak - vExit * vExit) / (2.0 * accel)
									 : 0.0;

			if (accelDistance + decelDistance > length)
			{
				const double term = std::max(0.0, accel * length + 0.5 * (vEntry * vEntry + vExit * vExit));
				vPeak = std::sqrt(term);
			}

			vPeak = std::max(vPeak, std::max(vEntry, vExit));
			vPeak = std::min(vPeak, plans[i].maxSpeed);
			plans[i].topSpeed = vPeak;
		}

		bool success = true;
		uint64_t& ticks = accumulated_ticks();

		for (const SegmentPlan& plan : plans)
		{
			float startCoords[MaxAxesPlusExtruders];
			for (size_t i = 0; i < MaxAxesPlusExtruders; ++i)
			{
				startCoords[i] = plan.startCoords[i];
			}

			DDA dda;
			if (!dda.Init(plan.move, startCoords))
			{
				success = false;
				break;
			}

			dda.SetPlannedProfile(static_cast<float>(plan.entrySpeed),
								  static_cast<float>(plan.topSpeed),
								  static_cast<float>(plan.exitSpeed),
								  static_cast<float>(plan.accelLimit));

			if (!dda.Prepare())
			{
				success = false;
				break;
			}

			const uint32_t startClock = static_cast<uint32_t>(ticks & 0xFFFFFFFFu);
			dda.SetMoveStartTime(startClock);
			const uint32_t clocksUsed = CanMotion::FinishMovement(dda, startClock, false);
			ticks += clocksUsed;
		}

		queue.clear();
		return success;
	}

} // namespace host::planner

#endif // RRF_HOST_BUILD
