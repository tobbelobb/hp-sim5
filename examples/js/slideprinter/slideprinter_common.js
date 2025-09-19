import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
    PositionComponent,
    RadiusComponent,
    VelocityComponent,
    PrevFinalPosComponent,
    RenderableComponent,
    OrientationComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
} from '../../../src/js/cable_joints/ecs.js';
import {
    CableLinkComponent,
    CableJointComponent,
    CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';

export class ExtruderComponent {
    constructor() {
        // Array of [[x,y,z], length]
        this.extrusions = [];
        this.centerPos = new Vector2(0.0, 0.0);
    }
}

export class ExtruderSystem {
    update(world, dt) {
        let extruderComp = null;
        for (const e of world.query([ExtruderComponent])) {
            extruderComp = world.getComponent(e, ExtruderComponent);
            break;
        }

        if (!extruderComp) {
            return;
        }

        const spoolPositions = [];
        const spoolEntities = world.query([SpoolTagComponent, PositionComponent]);
        for (const e of spoolEntities) {
            const pos = world.getComponent(e, PositionComponent).pos;
            spoolPositions.push(pos);
        }

        if (spoolPositions.length > 0) {
            const sum = new Vector2();
            spoolPositions.forEach(p => sum.add(p));
            extruderComp.centerPos = sum.scale(1 / spoolPositions.length);
        }
    }
}

export class SpoolTagComponent {}

export class SpoolStateComponent {
    constructor(axis = null) {
        this.axis = axis;
    }
}

// Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar
// TODO: This should be specced in the slideprinter.usda file, not hard coded in a .py file
export class StepperMotorComponent {
    constructor(
        commandedAngle = 0.0,
        deltaAngle = 0.0,
        holdingTorque = 0.5, // Nm. Within the typical range for Nema 17 motors
        numPolePairs = 50,   // For a 1.8 deg/step motor
        dampingCoeff = 0.01  // Gotten by trial and error. For stepper inertia 5e-05
    ) {
        this.commandedAngle = commandedAngle;
        this.deltaAngle = deltaAngle;
        this.holdingTorque = holdingTorque;
        this.numPolePairs = numPolePairs;
        this.dampingCoeff = dampingCoeff;
    }
}

// Adapted from the Stepper Motor Model for Dynamic Simulation paper by Alexandru Morar
export class StepperMotorSystem {
    update(world, dt) {
        const query = [
            StepperMotorComponent,
            OrientationComponent,
            AngularVelocityComponent,
            MomentOfInertiaComponent,
        ];
        for (const e of world.query(query)) {
            const stepper = world.getComponent(e, StepperMotorComponent);
            const orient = world.getComponent(e, OrientationComponent);
            const angVel = world.getComponent(e, AngularVelocityComponent);
            const inertia = world.getComponent(e, MomentOfInertiaComponent);

            // Calculate restoring torque based on angular error
            const error = orient.angle - (stepper.commandedAngle - stepper.deltaAngle);
            const restoringTorque = -stepper.holdingTorque * Math.sin(stepper.numPolePairs * error);

            // Add damping to help the motor settle
            const dampingTorque = -stepper.dampingCoeff * angVel.angularVelocity;

            const totalTorque = restoringTorque + dampingTorque;

            // Apply torque to angular velocity (F=ma -> a=F/m -> v=v+a*dt)
            const angularAcceleration = totalTorque / inertia.inertia;
            angVel.angularVelocity += angularAcceleration * dt;
            //orient.angle = stepper.commandedAngle;
        }
    }
}

export class RemoteSpoolSystem {
    constructor() {
        this.commands = [];
        this.axisToEntity = {};
        this.worker = null;
        this.wasPaused = false;
        this.highWaterMark = 40;
        this.lowWaterMark = 20;
    }

    addCommand(command) {
        this.commands.push(command);
    }

    update(world, dt) {
        if (this.worker) {
            const queueSize = this.commands.length;
            if (queueSize > this.highWaterMark && !this.wasPaused) {
                this.worker.postMessage({ type: 'pause' });
                this.wasPaused = true;
            } else if (queueSize < this.lowWaterMark && this.wasPaused) {
                this.worker.postMessage({ type: 'resume' });
                this.wasPaused = false;
            }
        }

        // Cache axis-to-entity mapping if not already done
        if (Object.keys(this.axisToEntity).length === 0) {
            const spoolEntities = world.query([SpoolTagComponent, SpoolStateComponent]);
            for (const e of spoolEntities) {
                const state = world.getComponent(e, SpoolStateComponent);
                if (state.axis) {
                    this.axisToEntity[state.axis] = e;
                }
            }
        }

        const command = this.commands.shift();
        if (command === undefined) {
            return;
        }

        if (command.E !== undefined && command.E > 0.0) {
            let extruderComp = null;
            for (const e of world.query([ExtruderComponent])) {
                extruderComp = world.getComponent(e, ExtruderComponent);
                break;
            }
            if (extruderComp != null) {
                const extrusionEvent = [[extruderComp.centerPos.x, extruderComp.centerPos.y, 0], command.E];
                extruderComp.extrusions.push(extrusionEvent);
            }
        }

        for (const axis in this.axisToEntity) {
            const entityId = this.axisToEntity[axis];
            if (command && command.type === 'Move' && command[axis] !== undefined) {
                const stepperComp = world.getComponent(entityId, StepperMotorComponent);
                if (stepperComp != null) {
                    stepperComp.commandedAngle = command[axis];
                }
            }
            if (command != null && command.type === 'Add to reference' && command[axis] !== undefined) {
                const stepperComp = world.getComponent(entityId, StepperMotorComponent);
                if (stepperComp) {
                    stepperComp.deltaAngle += command[axis];
                }
            }
        }
    }
}

// --- System: Remote Input ---
export class RemoteInputSystem {
     runInPause = true;

     constructor(canvas, world, ws) {
         this.canvas = canvas;
         this.world = world;
         this.ws = ws;
         this.scaleMultiplier = 1.0;
         this.viewOffsetX = 0.0;
         this.viewOffsetY = 0.0;
         this.interactionMode = 'select';
         this.isPanning = false;
         this.panPointerId = null;
         this.panLastX = 0;
         this.panLastY = 0;
         this.onViewChange = null;

         this.handlePointerDown = this.handlePointerDown.bind(this);
         this.handlePointerMove = this.handlePointerMove.bind(this);
         this.handlePointerUp = this.handlePointerUp.bind(this);

         document.addEventListener('pointerdown', this.handlePointerDown);
         document.addEventListener('pointermove', this.handlePointerMove);
         document.addEventListener('pointerup', this.handlePointerUp);
     }

     setInteractionMode(mode) {
         this.interactionMode = mode === 'pan' ? 'pan' : 'select';
         if (this.interactionMode !== 'pan') {
             this.isPanning = false;
             this.panPointerId = null;
         }
     }

     setViewTransform({ scaleMultiplier, offsetX, offsetY }) {
         if (typeof scaleMultiplier === 'number') {
             this.scaleMultiplier = scaleMultiplier;
         }
         if (typeof offsetX === 'number') {
             this.viewOffsetX = offsetX;
         }
         if (typeof offsetY === 'number') {
             this.viewOffsetY = offsetY;
         }
     }

     setViewChangeListener(listener) {
         this.onViewChange = typeof listener === 'function' ? listener : null;
     }

     toSimCoords(canvasX, canvasY) {
        const rect = this.canvas.getBoundingClientRect();
        const baseScale = this.canvas.height / this.world.getResource('simHeight');
        const scale = baseScale * this.scaleMultiplier;
        const pixelX = canvasX - rect.left;
        const pixelY = canvasY - rect.top;
        const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
        const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;
        return { x: simX, y: simY };
     }

     handlePointerDown(event) {
         event.preventDefault();
         const onCanvas = event.target === this.canvas;
         if (onCanvas && this.interactionMode === 'pan') {
             this.isPanning = true;
             this.panPointerId = event.pointerId;
             this.panLastX = event.clientX;
             this.panLastY = event.clientY;
             if (typeof this.canvas.setPointerCapture === 'function') {
                 try {
                     this.canvas.setPointerCapture(event.pointerId);
                 } catch (err) {
                     // ignore capture errors
                 }
             }
             return;
         }

         if (!onCanvas) {
             return;
         }

         if (this.ws && this.ws.readyState === WebSocket.OPEN) {
             const { x, y } = this.toSimCoords(event.clientX, event.clientY);

             let isGrabClick = false;
             for (const spoolId of this.world.query([SpoolTagComponent, PositionComponent, RadiusComponent])) {
                 const pos = this.world.getComponent(spoolId, PositionComponent).pos;
                 const radius = this.world.getComponent(spoolId, RadiusComponent).radius;
                 const dx = x - pos.x;
                 const dy = y - pos.y;
                 if (dx * dx + dy * dy <= radius * radius) {
                     isGrabClick = true;
                     break;
                 }
             }

             if (isGrabClick) {
                 const pauseState = this.world.getResource('pauseState');
                 if (pauseState && pauseState.paused) {
                     pauseState.paused = false;
                     const pauseBtn = document.getElementById("pauseBtn");
                     if (pauseBtn) pauseBtn.textContent = "Pause";
                     this.ws.send(JSON.stringify({ action: 'pause', paused: false }));
                 }
             }

            this.ws.send(JSON.stringify({ action: 'input', type: 'pointerdown', x, y }));
        }
        if (typeof this.canvas.setPointerCapture === 'function') {
            try {
                this.canvas.setPointerCapture(event.pointerId);
            } catch (err) {
                // ignore capture errors
            }
        }
    }

     handlePointerMove(event) {
         if (this.interactionMode === 'pan') {
             if (!this.isPanning || this.panPointerId !== event.pointerId) {
                 return;
             }
             event.preventDefault();
             const baseScale = this.canvas.height / this.world.getResource('simHeight');
             const scale = baseScale * this.scaleMultiplier;
             if (scale <= 0) {
                 return;
             }
             const deltaX = event.clientX - this.panLastX;
             const deltaY = event.clientY - this.panLastY;
             this.panLastX = event.clientX;
             this.panLastY = event.clientY;
             this.viewOffsetX -= deltaX / scale;
             this.viewOffsetY += deltaY / scale;
             if (this.onViewChange) {
                 this.onViewChange({
                     scale: this.scaleMultiplier,
                     offsetX: this.viewOffsetX,
                     offsetY: this.viewOffsetY,
                 });
             }
             return;
         }

         if (event.target !== this.canvas) {
             return;
         }

         event.preventDefault();
         if (this.ws && this.ws.readyState === WebSocket.OPEN) {
             const { x, y } = this.toSimCoords(event.clientX, event.clientY);
             this.ws.send(JSON.stringify({ action: 'input', type: 'pointermove', x, y }));
         }
     }

     handlePointerUp(event) {
         event.preventDefault();
         if (this.interactionMode === 'pan') {
             if (this.isPanning && this.panPointerId === event.pointerId) {
                 this.isPanning = false;
                 this.panPointerId = null;
                 if (typeof this.canvas.releasePointerCapture === 'function') {
                     try {
                         this.canvas.releasePointerCapture(event.pointerId);
                     } catch (err) {
                         // ignore release errors
                     }
                 }
             }
             return;
         }

         if (event.target !== this.canvas) {
             return;
         }

         if (this.ws && this.ws.readyState === WebSocket.OPEN) {
             const { x, y } = this.toSimCoords(event.clientX, event.clientY);
             this.ws.send(JSON.stringify({ action: 'input', type: 'pointerup', x, y }));
         }
         if (typeof this.canvas.releasePointerCapture === 'function') {
             try {
                 this.canvas.releasePointerCapture(event.pointerId);
             } catch (err) {
                 // ignore release errors
             }
         }
     }

     update(world, dt) {
        // This system doesn't run per-step logic, only handles events.
     }
}

// --- System: Input --- (Simplified Click Handling)
export class InputSystem {
     runInPause = true; // Input should work even when paused to unpause/interact

     constructor(canvas, world, pauseBtn) {
         this.canvas = canvas;
         this.world = world;
         this.pauseBtn = pauseBtn;
         this.clicks = [];
         this.releases = [];
         this.eventLog = [];
         this.frame = 0;
         this.grabSpring = null; // { ptrE, jointE, pathE, ballE }
         this.scaleMultiplier = 1.0;
         this.viewOffsetX = 0.0;
         this.viewOffsetY = 0.0;
         this.interactionMode = 'select';
         this.isPanning = false;
         this.panPointerId = null;
         this.panLastX = 0;
         this.panLastY = 0;
         this.onViewChange = null;
         this.canvas.setAttribute('tabindex', '0');
         this.canvas.style.outline = 'none';
         this.canvas.focus();
         this.touchActionBeforeGrab = null;
         this.activeGrabPointerId = null;
         this.scrollBlockerAttached = false;
         this.touchMoveListenerOptions = { passive: false };
         this.preventScrollDuringGrab = (event) => {
             if (this.activeGrabPointerId !== null) {
                 event.preventDefault();
             }
         };
         document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
         document.addEventListener('pointerup', this.handlePointerUp.bind(this));
         document.addEventListener('pointercancel', this.handlePointerCancel.bind(this));
         this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
     }

     setInteractionMode(mode) {
         this.interactionMode = mode === 'pan' ? 'pan' : 'select';
         if (this.interactionMode !== 'pan' && this.isPanning) {
             this.isPanning = false;
             this.panPointerId = null;
         }
     }

     setViewTransform({ scaleMultiplier, offsetX, offsetY }) {
         if (typeof scaleMultiplier === 'number') {
             this.scaleMultiplier = scaleMultiplier;
         }
         if (typeof offsetX === 'number') {
             this.viewOffsetX = offsetX;
         }
         if (typeof offsetY === 'number') {
             this.viewOffsetY = offsetY;
         }
     }

     setViewChangeListener(listener) {
         this.onViewChange = typeof listener === 'function' ? listener : null;
     }

     reset() {
        this.clicks = [];
        this.releases = [];
        this.eventLog = [];
        this.frame = 0;
        this.isPanning = false;
        this.panPointerId = null;
        this.activeGrabPointerId = null;
        this.setTouchScrollBlockActive(false);
        if (this.touchActionBeforeGrab !== null) {
            this.canvas.style.touchAction = this.touchActionBeforeGrab;
            this.touchActionBeforeGrab = null;
        }
        if (this.grabSpring) {
            const { ptrE, jointE, pathE } = this.grabSpring;
            this.world.destroyEntity(pathE);
            this.world.destroyEntity(jointE);
            this.world.destroyEntity(ptrE);
            this.grabSpring = null;
        }
     }

     setTouchScrollBlockActive(active) {
         if (typeof window === 'undefined' || !('ontouchstart' in window)) {
             return;
         }
         if (active) {
             if (!this.scrollBlockerAttached) {
                 document.addEventListener('touchmove', this.preventScrollDuringGrab, this.touchMoveListenerOptions);
                 this.scrollBlockerAttached = true;
             }
         } else if (this.scrollBlockerAttached) {
             document.removeEventListener('touchmove', this.preventScrollDuringGrab, this.touchMoveListenerOptions);
             this.scrollBlockerAttached = false;
         }
     }

     handlePointerDown(event) {
         if (event.target !== this.canvas) return;
         event.preventDefault();
         if (this.interactionMode === 'pan') {
             this.isPanning = true;
             this.panPointerId = event.pointerId;
             this.panLastX = event.clientX;
             this.panLastY = event.clientY;
             if (typeof this.canvas.setPointerCapture === 'function') {
                 try {
                     this.canvas.setPointerCapture(event.pointerId);
                 } catch (err) {
                     // Ignore browsers that disallow capture here.
                 }
             }
             return;
         }
         const rect = this.canvas.getBoundingClientRect();
         const baseScale = this.canvas.height / this.world.getResource('simHeight');
         const scale = baseScale * this.scaleMultiplier;
         const pixelX = event.clientX - rect.left;
         const pixelY = event.clientY - rect.top;
         const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
         const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;
         const clickVec = new Vector2(simX, simY);

         const cmOnScreen = 0.5;
         const dpi = 96;
         const pixelsPerCm = dpi / 2.54;
         const extraPixels = cmOnScreen * pixelsPerCm;
         const effectiveScale = (this.canvas.height / this.world.getResource('simHeight')) * this.scaleMultiplier;
         const extraClickableRadius = extraPixels / effectiveScale;

         let closestBall = null;
         let closestDistSq = Infinity;
         for (const b of this.world.query([SpoolTagComponent, PositionComponent, RadiusComponent])) {
           const pos = this.world.getComponent(b, PositionComponent).pos;
           const r = this.world.getComponent(b, RadiusComponent).radius + extraClickableRadius;
           const distSq = clickVec.clone().subtract(pos).lengthSq();
           if (distSq <= r * r && distSq < closestDistSq) {
             closestBall = b;
             closestDistSq = distSq;
           }
         }

         if (closestBall !== null) {
           if ((event.pointerType === 'touch' || event.pointerType === 'pen') && this.interactionMode !== 'pan') {
             if (this.touchActionBeforeGrab === null) {
               this.touchActionBeforeGrab = this.canvas.style.touchAction;
             }
             this.canvas.style.touchAction = 'none';
             this.activeGrabPointerId = event.pointerId;
             this.setTouchScrollBlockActive(true);
           }
           const ptrE = this.world.createEntity();
           this.world.addComponent(ptrE, new PositionComponent(simX, simY));
           this.world.addComponent(ptrE, new CableLinkComponent(simX, simY));

           const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();
           const ptrPos  = new Vector2(simX, simY);
           const jointE = this.world.createEntity();
           this.world.addComponent(jointE,
             new CableJointComponent(
               closestBall, ptrE, 0.1,
               ballPos.clone().subtract(ballPos), // local attachment on ball is zero
               ptrPos.clone().subtract(ptrPos) // local attachment on pointer is zero
             )
           );
           this.world.addComponent(jointE, new RenderableComponent('line', '#888888'));

           const pathE = this.world.createEntity();
           const pathComp = new CablePathComponent(
             this.world, [ jointE ], ['attachment', 'attachment'], [ true, true ], 10.0
           );
           this.world.addComponent(pathE, pathComp);

           this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
           this.world.setResource('grabbedBall', closestBall);

           const pauseState = this.world.getResource('pauseState');
           if (pauseState) {
             pauseState.paused = false;
           }
           if (this.pauseBtn) {
             this.pauseBtn.textContent = "Pause";
           }
           if (typeof this.canvas.setPointerCapture === 'function') {
             try {
               this.canvas.setPointerCapture(event.pointerId);
             } catch (err) {
               // Ignore browsers that disallow capture here.
             }
           }
           return;
         }
     }

     handlePointerUp(event) {
         if (event.target !== this.canvas) return;
         event.preventDefault();
         if (this.interactionMode === 'pan') {
             if (this.isPanning && this.panPointerId === event.pointerId) {
                 this.isPanning = false;
                 this.panPointerId = null;
                 if (typeof this.canvas.releasePointerCapture === 'function') {
                     try {
                         this.canvas.releasePointerCapture(event.pointerId);
                     } catch (err) {
                         // Ignore errors when capture was not set.
                     }
                 }
             }
             return;
        }
        if (typeof this.canvas.releasePointerCapture === 'function') {
            try {
                this.canvas.releasePointerCapture(event.pointerId);
            } catch (err) {
                // Ignore errors if capture was never set.
            }
        }

         if (this.grabSpring) {
           const { ptrE, jointE, pathE, ballE } = this.grabSpring;

           const posComp = this.world.getComponent(ballE, PositionComponent);
           const velComp = this.world.getComponent(ballE, VelocityComponent);
           const prevFinalPosComp = this.world.getComponent(ballE, PrevFinalPosComponent);
           const dt = this.world.getResource('dt');

           if (velComp && posComp && prevFinalPosComp && dt > 1e-9) {
             velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0/dt));
           } else if (velComp) {
             velComp.vel.set(new Vector2(0,0));
           }

           this.world.destroyEntity(pathE);
           this.world.destroyEntity(jointE);
           this.world.destroyEntity(ptrE);
           this.grabSpring = null;
           this.world.setResource('grabbedBall', null);
           if (this.touchActionBeforeGrab !== null && this.interactionMode !== 'pan') {
             this.canvas.style.touchAction = this.touchActionBeforeGrab;
             this.touchActionBeforeGrab = null;
           }
         }
         if (this.activeGrabPointerId === event.pointerId) {
           this.activeGrabPointerId = null;
           this.setTouchScrollBlockActive(false);
         }
     }

     handlePointerCancel(event) {
         this.handlePointerUp(event);
     }

     update(world, dt) {
        // This system doesn't run per-step logic, only handles events.
     }

     handlePointerMove(event) {
      if (event.target !== this.canvas) {
          return;
      }

      if (this.interactionMode === 'pan') {
          if (!this.isPanning || this.panPointerId !== event.pointerId) {
              return;
          }
          event.preventDefault();
          const baseScale = this.canvas.height / this.world.getResource('simHeight');
          const scale = baseScale * this.scaleMultiplier;
          if (scale <= 0) {
              return;
          }
          const deltaX = event.clientX - this.panLastX;
          const deltaY = event.clientY - this.panLastY;
          this.panLastX = event.clientX;
          this.panLastY = event.clientY;
          this.viewOffsetX -= deltaX / scale;
          this.viewOffsetY += deltaY / scale;
          if (this.onViewChange) {
              this.onViewChange({
                  scale: this.scaleMultiplier,
                  offsetX: this.viewOffsetX,
                  offsetY: this.viewOffsetY,
              });
          }
          return;
      }

      event.preventDefault();
      if (this.grabSpring === null) return;

      const rect = this.canvas.getBoundingClientRect();
      const baseScale = this.canvas.height / this.world.getResource('simHeight');
      const scale = baseScale * this.scaleMultiplier;
      const pixelX = event.clientX - rect.left;
      const pixelY = event.clientY - rect.top;
      const simX = (pixelX - this.canvas.width / 2) / scale + this.viewOffsetX;
      const simY = (this.canvas.height / 2 - pixelY) / scale + this.viewOffsetY;

      const { ptrE } = this.grabSpring;
      const pos = this.world.getComponent(ptrE, PositionComponent).pos;
      pos.set(new Vector2(simX, simY));
     }
}
