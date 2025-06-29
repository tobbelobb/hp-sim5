import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
    PositionComponent,
    RadiusComponent,
    VelocityComponent,
    PrevFinalPosComponent,
    RenderableComponent,
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
        this.centerPos = [0.0, 0.0, 0.0];
    }
}

export class SpoolTagComponent {}

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
         this.canvas.setAttribute('tabindex', '0');
         this.canvas.style.outline = 'none';
         this.canvas.focus();
         document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
         document.addEventListener('pointerup', this.handlePointerUp.bind(this));
         this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
     }

     reset() {
        this.clicks = [];
        this.releases = [];
        this.eventLog = [];
        this.frame = 0;
        if (this.grabSpring) {
            const { ptrE, jointE, pathE } = this.grabSpring;
            this.world.destroyEntity(pathE);
            this.world.destroyEntity(jointE);
            this.world.destroyEntity(ptrE);
            this.grabSpring = null;
        }
     }

     handlePointerDown(event) {
         event.preventDefault();
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
           pauseState.paused = false;
           this.pauseBtn.textContent = "Pause";
           return;
         }
     }

     handlePointerUp(event) {
         event.preventDefault();
         this.canvas.releasePointerCapture(event.pointerId);

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
         }
     }

     update(world, dt) {
        // This system doesn't run per-step logic, only handles events.
     }

     handlePointerMove(event) {
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
