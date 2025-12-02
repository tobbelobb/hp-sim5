# Subtask 6: Torque Mode in Physics Engine

## Overview
Extend the physics simulation to support torque mode for stepper motors. When in torque mode:
- The motor applies a constant torque (instead of position-seeking behavior)
- The spool/axis can rotate freely under external forces
- Torque direction affects which way the motor pulls

## Files to Modify

- `examples/js/slideprinter/slideprinter_common.js` - StepperMotorComponent and StepperMotorSystem

## Implementation Details

### 1. Extend StepperMotorComponent

Add torque mode state to the component:

```javascript
// In slideprinter_common.js, around line 142

export class StepperMotorComponent {
    constructor(
        commandedAngle = 0.0,
        deltaAngle = 0.0,
        holdingTorque = 0.5,     // Nm. Within the typical range for Nema 17 motors
        numPolePairs = 50,       // For a 1.8 deg/step motor
        dampingCoeff = 0.01      // Gotten by trial and error
    ) {
        this.commandedAngle = commandedAngle;
        this.deltaAngle = deltaAngle;
        this.holdingTorque = holdingTorque;
        this.numPolePairs = numPolePairs;
        this.dampingCoeff = dampingCoeff;

        // NEW: Torque mode support
        this.torqueMode = false;      // true = constant torque, false = position control
        this.targetTorque = 0.0;      // Nm, only used when torqueMode = true
    }
}
```

### 2. Modify StepperMotorSystem

Update the physics update to handle torque mode:

```javascript
// In slideprinter_common.js, replace StepperMotorSystem class (around line 159)

export class StepperMotorSystem {
    update(world, dt) {
        const query = [
            StepperMotorComponent,
            OrientationComponent,
            AngularVelocityComponent,
            MomentOfInertiaComponent,
        ];

        // Build a quick lookup: entityId -> group angle (if in a rigid group)
        const groupAngleByMember = new Map();
        try {
            const groups = world.query([RigidGroupComponent]);
            for (const gid of groups) {
                const group = world.getComponent(gid, RigidGroupComponent);
                const angle = group?.prevAngle || 0.0;
                const members = group?.members || [];
                for (const m of members) {
                    groupAngleByMember.set(m, angle);
                }
            }
        } catch(_e) {
            // RigidGroupComponent may not be present; ignore
        }

        for (const e of world.query(query)) {
            const stepper = world.getComponent(e, StepperMotorComponent);
            const orient = world.getComponent(e, OrientationComponent);
            const angVel = world.getComponent(e, AngularVelocityComponent);
            const inertia = world.getComponent(e, MomentOfInertiaComponent);

            let totalTorque;

            if (stepper.torqueMode) {
                // TORQUE MODE: Apply constant torque
                // The motor applies targetTorque regardless of position
                totalTorque = stepper.targetTorque;

                // Still apply damping to prevent runaway
                const dampingTorque = -stepper.dampingCoeff * angVel.angularVelocity;
                totalTorque += dampingTorque;
            } else {
                // POSITION MODE: Calculate restoring torque based on angular error
                // Make the stepper act in the group's local frame so rigid-group rotation
                // does not fight the motor controller
                const groupAngle = groupAngleByMember.get(e) || 0.0;
                const targetWorldAngle = groupAngle + (stepper.commandedAngle - stepper.deltaAngle);
                const error = orient.angle - targetWorldAngle;
                const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);

                // Add damping to help the motor settle
                const dampingTorque = -stepper.dampingCoeff * angVel.angularVelocity;

                totalTorque = restoringTorque + dampingTorque;
            }

            // Apply torque to angular velocity (F=ma -> a=F/m -> v=v+a*dt)
            const angularAcceleration = totalTorque / inertia.inertia;
            angVel.angularVelocity += angularAcceleration * dt;
        }
    }
}
```

### 3. Helper Functions for Torque Mode Control

Add utility functions for external control:

```javascript
// In slideprinter_common.js, after StepperMotorSystem

/**
 * Set a stepper motor to torque mode.
 * @param {World} world - ECS world
 * @param {Entity} entity - Motor entity
 * @param {number} torqueNm - Torque to apply in Nm
 */
export function setStepperTorqueMode(world, entity, torqueNm) {
    const stepper = world.getComponent(entity, StepperMotorComponent);
    if (stepper) {
        stepper.torqueMode = true;
        stepper.targetTorque = torqueNm;
    }
}

/**
 * Set a stepper motor back to position mode.
 * @param {World} world - ECS world
 * @param {Entity} entity - Motor entity
 */
export function setStepperPositionMode(world, entity) {
    const stepper = world.getComponent(entity, StepperMotorComponent);
    if (stepper) {
        stepper.torqueMode = false;
        stepper.targetTorque = 0;
    }
}

/**
 * Check if a stepper is in torque mode.
 * @param {World} world - ECS world
 * @param {Entity} entity - Motor entity
 * @returns {boolean}
 */
export function isStepperInTorqueMode(world, entity) {
    const stepper = world.getComponent(entity, StepperMotorComponent);
    return stepper?.torqueMode ?? false;
}

/**
 * Get the current torque setting for a motor.
 * @param {World} world - ECS world
 * @param {Entity} entity - Motor entity
 * @returns {number} Torque in Nm (0 if in position mode)
 */
export function getStepperTorque(world, entity) {
    const stepper = world.getComponent(entity, StepperMotorComponent);
    return stepper?.torqueMode ? stepper.targetTorque : 0;
}
```

### 4. Update RemoteSpoolSystem Command Handler

Ensure torque mode commands work through the command queue:

```javascript
// In RemoteSpoolSystem._processCommand(), add after existing command handlers

if (commandType === 'SetTorqueMode') {
    const axis = command.axis;
    const entities = this.axisToEntity[axis];

    if (entities && entities.length > 0) {
        for (const entity of entities) {
            setStepperTorqueMode(world, entity, command.torqueNm);
        }

        if (this.onCommandExecuted) {
            this.onCommandExecuted(command, simTime);
        }
        return true;
    }
    return false;
}

if (commandType === 'SetPositionMode') {
    const axis = command.axis;
    const entities = this.axisToEntity[axis];

    if (entities && entities.length > 0) {
        for (const entity of entities) {
            setStepperPositionMode(world, entity);
        }

        if (this.onCommandExecuted) {
            this.onCommandExecuted(command, simTime);
        }
        return true;
    }
    return false;
}
```

### 5. Torque Sign Conventions

The real Hangprinter firmware applies sign corrections for certain drivers:

```cpp
// From HangprinterKinematics.cpp line 1355
if (driver.boardAddress == 40 || driver.boardAddress == 41) {
    motorTorque_Nm = -motorTorque_Nm;
}
```

In the physics sim, positive torque should wind the spool (shorten line), negative should unwind. The sign mapping depends on how motors are mounted in the specific machine. For the simulation, we can:

Option A: Apply the same sign correction in the physics engine
Option B: Let the firmware/HOST_BUILD handler do the correction (simpler)

**Recommendation**: Apply correction on the firmware side (Subtask 3) so the physics engine receives correctly-signed values.

### 6. Physical Behavior Under Torque Mode

When in torque mode with a small positive torque:
1. Motor applies constant pulling force
2. If external forces (gravity, payload) oppose, motor holds at equilibrium
3. If external forces align, spool winds up
4. Line tension increases or decreases based on net torque

This is useful for:
- Tightening slack lines
- Measuring line tension (by observing when motion stops)
- Auto-calibration procedures

## Testing

### Unit Tests

```javascript
// examples/js/slideprinter/tests/torqueMode.test.js

import {
    StepperMotorComponent,
    StepperMotorSystem,
    setStepperTorqueMode,
    setStepperPositionMode,
    isStepperInTorqueMode
} from '../slideprinter_common.js';

describe('Torque Mode', () => {
    let world;
    let motorEntity;
    let system;

    beforeEach(() => {
        // Create minimal ECS world mock
        world = createMockWorld();
        motorEntity = world.createEntity();
        world.addComponent(motorEntity, StepperMotorComponent, new StepperMotorComponent());
        world.addComponent(motorEntity, OrientationComponent, { angle: 0 });
        world.addComponent(motorEntity, AngularVelocityComponent, { angularVelocity: 0 });
        world.addComponent(motorEntity, MomentOfInertiaComponent, { inertia: 0.00005 });

        system = new StepperMotorSystem();
    });

    it('starts in position mode', () => {
        expect(isStepperInTorqueMode(world, motorEntity)).toBe(false);
    });

    it('can switch to torque mode', () => {
        setStepperTorqueMode(world, motorEntity, 0.001);

        expect(isStepperInTorqueMode(world, motorEntity)).toBe(true);

        const stepper = world.getComponent(motorEntity, StepperMotorComponent);
        expect(stepper.targetTorque).toBeCloseTo(0.001);
    });

    it('can switch back to position mode', () => {
        setStepperTorqueMode(world, motorEntity, 0.001);
        setStepperPositionMode(world, motorEntity);

        expect(isStepperInTorqueMode(world, motorEntity)).toBe(false);
    });

    it('applies constant torque in torque mode', () => {
        setStepperTorqueMode(world, motorEntity, 0.01);  // 0.01 Nm

        const angVel = world.getComponent(motorEntity, AngularVelocityComponent);
        const initialVelocity = angVel.angularVelocity;

        // Run physics step
        system.update(world, 0.001);  // 1ms step

        // Velocity should increase (torque causes acceleration)
        expect(angVel.angularVelocity).toBeGreaterThan(initialVelocity);
    });

    it('applies restoring torque in position mode', () => {
        const orient = world.getComponent(motorEntity, OrientationComponent);
        orient.angle = 0.1;  // 0.1 rad away from target (0)

        const angVel = world.getComponent(motorEntity, AngularVelocityComponent);

        // Run physics step
        system.update(world, 0.001);

        // Motor should accelerate toward target (negative velocity to reduce angle)
        expect(angVel.angularVelocity).toBeLessThan(0);
    });

    it('damping limits velocity in torque mode', () => {
        setStepperTorqueMode(world, motorEntity, 0.001);

        const angVel = world.getComponent(motorEntity, AngularVelocityComponent);
        angVel.angularVelocity = 100;  // Very high initial velocity

        // Run many physics steps
        for (let i = 0; i < 1000; i++) {
            system.update(world, 0.001);
        }

        // Velocity should be bounded (not infinite)
        expect(Math.abs(angVel.angularVelocity)).toBeLessThan(1000);
    });
});
```

### Visual Test

Create a simple visual test that shows torque mode behavior:

```javascript
// examples/js/slideprinter/tests/torqueMode_visual.js

import { createWorld, createMotorEntity, StepperMotorSystem } from '../slideprinter_common.js';

const world = createWorld();
const motor = createMotorEntity(world);
const system = new StepperMotorSystem();

// Visual indicator
const canvas = document.createElement('canvas');
canvas.width = 400;
canvas.height = 400;
document.body.appendChild(canvas);
const ctx = canvas.getContext('2d');

let torqueMode = false;
let targetTorque = 0.001;

function draw() {
    const orient = world.getComponent(motor, OrientationComponent);
    const angle = orient.angle;

    ctx.clearRect(0, 0, 400, 400);

    // Draw spool
    ctx.save();
    ctx.translate(200, 200);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.arc(0, 0, 50, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(50, 0);
    ctx.stroke();
    ctx.restore();

    // Status
    ctx.fillText(`Mode: ${torqueMode ? 'TORQUE' : 'POSITION'}`, 10, 20);
    ctx.fillText(`Angle: ${angle.toFixed(3)} rad`, 10, 40);
    ctx.fillText(`Press T to toggle mode`, 10, 380);
}

function update() {
    system.update(world, 0.016);  // 60fps
    draw();
    requestAnimationFrame(update);
}

document.addEventListener('keydown', (e) => {
    if (e.key === 't' || e.key === 'T') {
        torqueMode = !torqueMode;
        const stepper = world.getComponent(motor, StepperMotorComponent);
        stepper.torqueMode = torqueMode;
        stepper.targetTorque = targetTorque;
    }
});

update();
```

## Validation Criteria

1. `StepperMotorComponent` has `torqueMode` and `targetTorque` fields
2. Position mode works as before (restoring torque based on error)
3. Torque mode applies constant torque regardless of position
4. Damping prevents runaway velocity in torque mode
5. `setStepperTorqueMode()` and `setStepperPositionMode()` work correctly
6. RemoteSpoolSystem handles SetTorqueMode/SetPositionMode commands
7. Physics behavior matches expectations for line tensioning

## Integration with RrfHttpBridge

When the HTTP bridge receives a torque mode event (from M569.4), it adds a command to RemoteSpoolSystem:

```javascript
// Already defined in Subtask 5
{
    type: 'SetTorqueMode',
    axis: 'A',
    driver: 40,
    torqueNm: 0.001,
    timestamp: Date.now()
}
```

RemoteSpoolSystem._processCommand() handles this and calls setStepperTorqueMode() on the appropriate entity.

## Dependencies

- Subtask 5 (JS Bridge) feeds commands to RemoteSpoolSystem
- Subtask 3 (M569.4 handler) generates the events

## Physical Accuracy Notes

The current stepper model uses:
```
restoringTorque = -holdingTorque * sin(numPolePairs * error)
```

This models the magnetic detent positions of a stepper. In torque mode, we override this with a constant value. For higher fidelity, consider:

1. **Back-EMF**: Fast rotation generates opposing voltage
2. **Current limiting**: Motor drivers limit current, capping torque
3. **Gear ratios**: Spool gear vs motor gear affects actual torque at spool

For the simulation's purpose (testing auto-calibration procedures), the simplified constant-torque model should be sufficient.

## Estimated Complexity
- Component extension: Simple (add two fields)
- System modification: Simple (add conditional branch)
- Helper functions: Simple
- Testing: Moderate (need physics verification)
