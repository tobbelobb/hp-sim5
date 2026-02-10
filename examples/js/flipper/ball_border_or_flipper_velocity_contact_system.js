import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
    PositionComponent,
    VelocityComponent,
    RadiusComponent,
    MassComponent,
    RestitutionComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
    CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';

import {
    FlipperStateComponent,
    FlipperTipComponent
} from './flipper_common.js';

export class BallBorderOrFlipperVelocityContactSystem {
    runInPause = false;

    _handleBallContact(
        world,
        ballId,
        normal,
        v_surface,
        restitution_other,
        friction_other,
        delta_lambda,
        dt,
        contactOffset = null
    ) {
        // Get all required components for the ball
        const posComp = world.getComponent(ballId, PositionComponent);
        const velComp = world.getComponent(ballId, VelocityComponent);
        const radiusComp = world.getComponent(ballId, RadiusComponent);
        const massComp = world.getComponent(ballId, MassComponent);
        const moiComp = world.getComponent(ballId, MomentOfInertiaComponent);
        const angVelComp = world.getComponent(ballId, AngularVelocityComponent);
        const restitutionComp = world.getComponent(ballId, RestitutionComponent);
        const frictionComp = world.getComponent(ballId, CoefficientOfFrictionComponent);

        if (!posComp || !velComp || !radiusComp || !massComp || !moiComp || !angVelComp || !restitutionComp) {
            return;
        }

        const mass = massComp.mass;
        const invMass = (mass > 0) ? 1.0 / mass : 0.0;
        const invInertia = moiComp.invInertia;

        if (invMass === 0.0 && invInertia === 0.0) {
            return;
        }

        const radius = radiusComp.radius;
        const restitutionBall = restitutionComp.restitution;
        const muBall = frictionComp ? frictionComp.mu : 0.0;
        const angVel = angVelComp.angularVelocity;

        const restitution = (restitutionBall + restitution_other) / 2.0;
        const mu = (muBall + friction_other) / 2.0;
        const useContactOffset =
            world.getResource('enableLayering') !== false &&
            world.getResource('layeringVelocityContactOffset') !== false;

        const useOffset =
            useContactOffset &&
            contactOffset &&
            Number.isFinite(contactOffset.x) &&
            Number.isFinite(contactOffset.y);
        const r_ball = useOffset
            ? new Vector2(contactOffset.x, contactOffset.y)
            : normal.clone().scale(-radius);

        const v_angular_at_contact = new Vector2(-angVel * r_ball.y, angVel * r_ball.x);
        const v_ball_at_contact = velComp.vel.clone().add(v_angular_at_contact);

        const v_rel = v_ball_at_contact.clone().subtract(v_surface);
        const v_rel_n_scalar = v_rel.dot(normal);

        // --- 1. Restitution (Normal Impulse) ---
        let j_n_restitution = 0;
        if (v_rel_n_scalar < 0) {
            const r_cross_n_z = r_ball.x * normal.y - r_ball.y * normal.x;
            const w_inv_n = invMass + invInertia * (r_cross_n_z ** 2);
            if (w_inv_n > 1e-9) {
                j_n_restitution = -(1.0 + restitution) * v_rel_n_scalar / w_inv_n;
                const impulse_n_vec = normal.clone().scale(j_n_restitution);
                velComp.vel.add(impulse_n_vec, invMass);
                const torque_n_impulse = r_cross_n_z * j_n_restitution;
                angVelComp.angularVelocity += torque_n_impulse * invInertia;
            }
        }

        // --- 2. Friction (Tangential Impulse) ---
        let j_n_force = 0;
        if (dt > 1e-9) {
            j_n_force = delta_lambda / dt;
        }
        const j_n_for_friction = j_n_restitution + j_n_force;

        const v_angular_at_contact_new = new Vector2(-angVelComp.angularVelocity * r_ball.y, angVelComp.angularVelocity * r_ball.x);
        const v_ball_at_contact_new = velComp.vel.clone().add(v_angular_at_contact_new);
        const v_rel_new = v_ball_at_contact_new.clone().subtract(v_surface);

        const v_rel_t_vec = v_rel_new.clone().subtract(normal.clone().scale(v_rel_new.dot(normal)));
        const v_rel_t_mag = v_rel_t_vec.length();

        if (mu > 0 && v_rel_t_mag > 1e-9) {
            const tangent = v_rel_t_vec.clone().scale(1.0 / v_rel_t_mag);
            const r_cross_t_z = r_ball.x * tangent.y - r_ball.y * tangent.x;
            const w_inv_t = invMass + invInertia * (r_cross_t_z ** 2);

            if (w_inv_t > 1e-9) {
                const j_t_noslip = -v_rel_t_mag / w_inv_t;
                const j_t = Math.max(-mu * j_n_for_friction, Math.min(j_t_noslip, mu * j_n_for_friction));
                const impulse_t_vec = tangent.clone().scale(j_t);
                velComp.vel.add(impulse_t_vec, invMass);
                const torque_t_impulse = r_cross_t_z * j_t;
                angVelComp.angularVelocity += torque_t_impulse * invInertia;
            }
        }
    }

    update(world, dt) {
        // --- Handle Border Contacts ---
        const borderContacts = world.getResource('ball_border_contacts');
        if (borderContacts) {
            for (const contact of borderContacts) {
                const restitutionBorder = contact.restitution;
                const frictionBorder = contact.friction;
                this._handleBallContact(
                    world,
                    contact.ball_id,
                    contact.normal,
                    new Vector2(0, 0),
                    restitutionBorder,
                    frictionBorder,
                    contact.delta_lambda,
                    dt,
                    contact.ball_contact_offset
                );
            }
        }

        // --- Handle Flipper Contacts ---
        const flipperContacts = world.getResource('ball_flipper_contacts');
        if (flipperContacts) {
            for (const contact of flipperContacts) {
                const { ball_id, flip_id, normal, contact_point_on_flipper, delta_lambda } = contact;

                const flipperPosComp = world.getComponent(flip_id, PositionComponent);
                const flipperStateComp = world.getComponent(flip_id, FlipperStateComponent);
                const flipperRestitutionComp = world.getComponent(flip_id, RestitutionComponent);

                let flipper_tip_id = null;
                const tipEntities = world.query([FlipperTipComponent]);
                for (const tipId of tipEntities) {
                    const tipComp = world.getComponent(tipId, FlipperTipComponent);
                    if (tipComp.flipperEntityId === flip_id) {
                        flipper_tip_id = tipId;
                        break;
                    }
                }
                const flipperFrictionComp = flipper_tip_id ? world.getComponent(flipper_tip_id, CoefficientOfFrictionComponent) : null;

                if (!flipperPosComp || !flipperStateComp || !flipperRestitutionComp) {
                    continue;
                }

                const flipperPivotPos = flipperPosComp.pos;
                const flipperAngVel = flipperStateComp.currentAngularVelocity;
                const r_flipper = contact_point_on_flipper.clone().subtract(flipperPivotPos);
                const v_flipper = new Vector2(-flipperAngVel * r_flipper.y, flipperAngVel * r_flipper.x);

                const restitutionFlipper = flipperRestitutionComp.restitution;
                const frictionFlipper = flipperFrictionComp ? flipperFrictionComp.mu : null;

                this._handleBallContact(
                    world,
                    ball_id,
                    normal,
                    v_flipper,
                    restitutionFlipper,
                    frictionFlipper,
                    delta_lambda,
                    dt,
                    contact.ball_contact_offset
                );
            }
        }
    }
}
