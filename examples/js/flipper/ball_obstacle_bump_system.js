import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
    VelocityComponent,
    RadiusComponent,
    MassComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
    CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';

import {
    ObstaclePushComponent,
    getEffectiveCollisionRadius
} from './flipper_common.js';

export class BallObstacleBumpSystem {
    runInPause = false;

    update(world, dt) {
        const contacts = world.getResource('ball_obstacle_contacts');
        if (!contacts || contacts.length === 0) {
            return;
        }

        for (const contact of contacts) {
            const { ball_id, obs_id, direction } = contact;
            if (contact.raw_contact === false) {
                continue;
            }

            const v1Comp = world.getComponent(ball_id, VelocityComponent);
            const r1Comp = world.getComponent(ball_id, RadiusComponent);
            const ballMassComp = world.getComponent(ball_id, MassComponent);
            const ballAngVelComp = world.getComponent(ball_id, AngularVelocityComponent);
            const ballMoiComp = world.getComponent(ball_id, MomentOfInertiaComponent);

            const r2Comp = world.getComponent(obs_id, RadiusComponent);
            const pushComp = world.getComponent(obs_id, ObstaclePushComponent);
            const obsAngVelComp = world.getComponent(obs_id, AngularVelocityComponent);
            const obsMoiComp = world.getComponent(obs_id, MomentOfInertiaComponent);
            const obsFrictionComp = world.getComponent(obs_id, CoefficientOfFrictionComponent);

            if (!v1Comp || !r1Comp || !ballMassComp || !r2Comp || !pushComp) {
                continue;
            }

            const v1 = v1Comp.vel;
            const r1 = Number.isFinite(contact.ball_contact_radius)
                ? contact.ball_contact_radius
                : getEffectiveCollisionRadius(world, ball_id, r1Comp.radius, direction.clone().scale(-1.0));
            const r2 = Number.isFinite(contact.obs_contact_radius)
                ? contact.obs_contact_radius
                : getEffectiveCollisionRadius(world, obs_id, r2Comp.radius, direction.clone());
            const pushVel = pushComp.pushVel;

            const omega_ball = ballAngVelComp ? ballAngVelComp.angularVelocity : 0.0;
            const omega_obs = obsAngVelComp ? obsAngVelComp.angularVelocity : 0.0;
            const mu = obsFrictionComp ? obsFrictionComp.mu : 0.0;

            const tangent = new Vector2(-direction.y, direction.x);

            // Obstacle surface velocity at contact point
            let v_surf_obs_tangential_comp = 0;
            if (omega_obs !== 0) {
                // v_surf = omega_obs x r_obs, where r_obs = direction * r2
                const v_surf_x = -omega_obs * direction.y * r2;
                const v_surf_y = omega_obs * direction.x * r2;
                v_surf_obs_tangential_comp = v_surf_x * tangent.x + v_surf_y * tangent.y;
            }

            // Ball surface velocity at contact point
            // v_contact_ball = v_ball + (omega_ball x r_ball), where r_ball = -direction * r1
            const v_angular_ball_at_contact = new Vector2(
                omega_ball * direction.y * r1,
                -omega_ball * direction.x * r1
            );
            const v_ball_at_contact = v1.clone().add(v_angular_ball_at_contact);
            const v_ball_tangential_comp = v_ball_at_contact.dot(tangent);

            const v_rel_tangential = v_ball_tangential_comp - v_surf_obs_tangential_comp;

            let s_sign = 0;
            // Friction opposes relative motion
            if (Math.abs(v_rel_tangential) > 1e-9 && mu !== 0) {
                s_sign = -Math.sign(v_rel_tangential);
            }

            let effectivePushDir = direction.clone();
            if (s_sign !== 0) {
                effectivePushDir.add(tangent.clone().scale(mu * s_sign));
            }
            effectivePushDir.normalize();

            v1.add(effectivePushDir, pushVel);

            if (mu > 0 && ballMassComp) {
                const delta_v_ball_tangential_actual_scalar_comp = effectivePushDir.dot(tangent) * pushVel;

                if (Math.abs(delta_v_ball_tangential_actual_scalar_comp) > 1e-9) {
                    const j_t_on_ball_vec = tangent.clone().scale(delta_v_ball_tangential_actual_scalar_comp * ballMassComp.mass);

                    if (ballAngVelComp && ballMoiComp && ballMoiComp.invInertia > 0) {
                        const r_contact_ball = direction.clone().scale(-r1);
                        const delta_l_ball = r_contact_ball.x * j_t_on_ball_vec.y - r_contact_ball.y * j_t_on_ball_vec.x;
                        ballAngVelComp.angularVelocity += delta_l_ball * ballMoiComp.invInertia;
                    }

                    if (obsAngVelComp && obsMoiComp && obsMoiComp.invInertia > 0) {
                        const j_t_on_obs_vec = j_t_on_ball_vec.clone().scale(-1);
                        const r_contact_obs = direction.clone().scale(r2);
                        const delta_l_obs = r_contact_obs.x * j_t_on_obs_vec.y - r_contact_obs.y * j_t_on_obs_vec.x;
                        obsAngVelComp.angularVelocity += delta_l_obs * obsMoiComp.invInertia;
                    }
                }
            }
        }
    }
}
