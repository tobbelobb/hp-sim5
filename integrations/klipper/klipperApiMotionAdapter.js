import { performance } from 'node:perf_hooks';
import {
  KlipperBucketedMotionCore,
  KlipperClockModel,
  mapStepperNameToAxis,
} from './klipperMotionCore.js';

export class KlipperApiMotionAdapter {
  constructor({
    now = () => performance.now(),
    onCommand = null,
    bucketIntervalMs = 2,
  } = {}) {
    this.now = now;
    this.onCommand = typeof onCommand === 'function' ? onCommand : null;
    this.clockModel = new KlipperClockModel();
    this.motionCore = new KlipperBucketedMotionCore({
      bucketIntervalMs,
    });
  }

  reset() {
    this.clockModel.reset();
    this.motionCore.reset();
  }

  updateClock(message, receiveMs = this.now()) {
    this.clockModel.updateFromMessage(message, receiveMs);
    return this.drainReadyCommands();
  }

  consumeStepperBatch(batch = {}) {
    const axis = batch.axis || mapStepperNameToAxis(batch.name);
    if (!axis) {
      return [];
    }
    const rawFirstClock = Number(batch.first_clock);
    const firstClock = this.clockModel.isReady()
      ? this.clockModel.unwrapRawClock(rawFirstClock)
      : rawFirstClock;
    this.motionCore.consumeStepperBatch({
      axis,
      firstClock,
      startMcuPosition: batch.start_mcu_position,
      data: batch.data,
    });
    return [];
  }

  drainReadyCommands({ force = false, ignoreClock = false } = {}) {
    const currentMcuTick = this.clockModel.getCurrentMcuTick();
    const maxKnownTick = this.motionCore.getMaxKnownTick();
    const shouldForce = force
      || (Number.isFinite(currentMcuTick)
        && Number.isFinite(maxKnownTick)
        && currentMcuTick >= maxKnownTick);
    const commands = this.motionCore.flushCommands({
      force: shouldForce,
      canEmitBucket: (bucketIdx) => {
        if (shouldForce || ignoreClock) {
          return true;
        }
        if (!Number.isFinite(currentMcuTick)) {
          return false;
        }
        return this.motionCore.getBucketEndTick(bucketIdx) <= currentMcuTick;
      },
      buildTiming: (bucketIdx) => {
        const bucketEndTick = this.motionCore.getBucketEndTick(bucketIdx);
        const at = ignoreClock
          ? this.now()
          : this.clockModel.unwrappedToWorkerMs(bucketEndTick);
        return {
          at: Number.isFinite(at) ? at : this.now(),
          span: this.clockModel.ticksToMs(this.motionCore.getTicksPerBucket()),
        };
      },
    });
    if (this.onCommand) {
      for (const command of commands) {
        this.onCommand(command);
      }
    }
    return commands;
  }
}
