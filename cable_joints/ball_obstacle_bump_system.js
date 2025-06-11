import Vector2 from './vector2.js';
import {
    VelocityComponent,
    RadiusComponent,
    MassComponent,
    AngularVelocityComponent,
    MomentOfInertiaComponent,
    CoefficientOfFrictionComponent,
    ObstaclePushComponent
} from './ecs.js';

export class BallObstacleBumpSystem {
    runInPause = false;

    update(world, dt) {
        const contacts = world.getResource('ball_obstacle_contacts');
        if (!contacts || contacts.length === 0) {
            return;
        }

        for (const contact of contacts) {
            const { ball_id, obs_id, direction } = contact;

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
            const r1 = r1Comp.radius;
            const r2 = r2Comp.radius;
            const pushVel = pushComp.pushVel;

            const omega_obs = obsAngVelComp ? obsAngVelComp.angularVelocity : 0.0;
            const mu = obsFrictionComp ? obsFrictionComp.mu : 0.0;

            const tangent = new Vector2(-direction.y, direction.x);

            let v_surf_obs_tangential_comp = 0;
            if (omega_obs !== 0) {
                const v_surf_x = -omega_obs * direction.y * r2;
                const v_surf_y = omega_obs * direction.x * r2;
                v_surf_obs_tangential_comp = v_surf_x * tangent.x + v_surf_y * tangent.y;
            }

            const s_sign = (v_surf_obs_tangential_comp === 0 || mu === 0) ? 0 : Math.sign(v_surf_obs_tangential_comp);
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
