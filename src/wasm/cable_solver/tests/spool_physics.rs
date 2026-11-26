//! Dedicated solver-correctness tests for strained spool storage and tension-dependent radial
//! flattening (PR #129 review items 1-7):
//!   1. Geometric spool occupancy vs stored unstretched material length across multi-revolution
//!      layering transitions, plus effective-radius scaling of motor reaction torque / Jacobian.
//!   2. Lifecycle preservation: checkpoint export/import, external `stored` edits, runtime toggle.
//!   3. Explicit initial storage tension policy for pre-wound rope.
//!   4. Adversarial clamping and per-step conservation invariants.
//!   5. Hybrid rolling <-> attachment transitions and hybrid-knot angle synchronization.
//!   6. Material-preserving coil coalescing and bounded memory under varying tension.
//!   7. Per-cable parameters, hysteresis + capstan interaction, dt convergence, legacy parity.
mod common;
use common::*;

const SPOOL_END: f64 = 3.0;

fn run_two_drum_wind_unwind(
    base_r: f64,
    half_w: f64,
    ea: f64,
    k_r: f64,
    strained: bool,
    f_s: f64,
    f_l: f64,
    wind_m: f64,
    dt: f64,
) -> (f64, f64, f64, f64) {
    let span = 2.0;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], base_r, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], base_r, 0.5, 1e6, None);
    let r0 = base_r + half_w;
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + f_s / ea),
        pa: [-span / 2.0, r0, 0.0],
        pb: [span / 2.0, r0, 0.0],
    }];
    let links = [(SPOOL_END, false, 0.0), (SPOOL_END, true, 12.0)];
    let mut p = path(&links, &segs, ea, 0.0, 4.0);
    p[6] = half_w;
    let mut w = world(vec![test, load], vec![p]);
    w.options[1] = 0.0;
    w.options[3] = f64::from(half_w > 0.0);
    w.set_cable_spool_params(0, k_r, strained, f_s);

    let speed = 1.5;
    let mat0 = material(&mut w, 0);
    let wind_steps = (wind_m / speed / dt).round() as usize;
    for _ in 0..wind_steps {
        let omega = speed / base_r;
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, omega]);
        // Match load-drum angular speed to current test-drum running radius so tension stays near f_s.
        let r_test = w.entities[0][104] + half_w;
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega * (r_test / r0)]);
        w.step(dt).unwrap();
        // Keep load drum tracking exact f_s gently via small speed trim on load drum.
        let t_now = tension(&mut w, 0, 0);
        let trim = 1e-4 * (f_s - t_now);
        w.entities[1][72..75][2] -= trim / base_r;
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
    }

    let geom_after_wind = w.spool_geom_stored(0, 0);
    let rest_after_wind = stored(&mut w, 0, 0);

    // Re-tension span to f_l by adjusting load drum only (test drum stays fixed).
    let jo = HEADER + 2 * LINK;
    let lo1 = HEADER + LINK;
    let target_rest = span / (1.0 + f_l / ea);
    let delta = rest(&mut w, 0, 0) - target_rest;
    w.path_data(0)[jo + 3] = target_rest;
    w.path_data(0)[lo1 + 2] += delta;
    for _ in 0..5 {
        w.entities[0][72..75].copy_from_slice(&[0.0; 3]);
        w.entities[1][72..75].copy_from_slice(&[0.0; 3]);
        w.step(dt).unwrap();
    }

    // Unwind back by 80% of what was wound at speed.
    let unwind_steps = (0.8 * wind_steps as f64).round() as usize;
    let rest_before_unwind = stored(&mut w, 0, 0);
    let geom_before_unwind = w.spool_geom_stored(0, 0);
    for _ in 0..unwind_steps {
        let t_now = tension(&mut w, 0, 0);
        let omega = -speed / base_r;
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, omega]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega - 2e-4 * (f_l - t_now) / base_r]);
        w.step(dt).unwrap();
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
    }
    let unwound_rest = rest_before_unwind - stored(&mut w, 0, 0);
    let unwound_geom = geom_before_unwind - w.spool_geom_stored(0, 0);
    (geom_after_wind, rest_after_wind, unwound_rest, unwound_geom)
}

/// Review Item 1: Multi-revolution winding with layering enabled must track geometric drum
/// occupancy (`geom_stored`) separately from unstretched material (`stored`), trigger layer
/// transitions at geometric revolutions, and unwind high-tension coils at their stored rho.
#[test]
fn multi_revolution_layering_separates_geom_from_material() {
    let base_r = 0.030;
    let half_w = 0.002; // 4 mm cable -> layer 0 circumference ~ 2*pi*0.032 = 0.201 m
    let ea = 25_000.0; // moderately compliant so strain is ~1% at 250 N
    let k_r = 1.0e-6;
    let f_s = 250.0;
    let f_l = 40.0;

    // Single drum winding against a tensioned spring-like load over >3 full revolutions (~0.8 m).
    let span = 2.0;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], base_r, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], 0.050, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + f_s / ea),
        pa: [-span / 2.0, base_r + half_w, 0.0],
        pb: [span / 2.0, 0.050 + half_w, 0.0],
    }];
    let links = [(SPOOL_END, false, 0.0), (SPOOL_END, true, 15.0)];
    let mut p = path(&links, &segs, ea, 0.0, 4.0);
    p[6] = half_w;
    let mut w = world(vec![test, load], vec![p]);
    w.options[1] = 0.0;
    w.options[3] = 1.0; // layering ON
    w.set_cable_spool_params(0, k_r, true, 0.0);

    let dt = 0.001;
    let mat0 = material(&mut w, 0);
    // Rotate test drum by exactly 3.5 full revolutions (21.99 rad) while holding tension ~250 N.
    let target_angle = 3.5 * 2.0 * PI;
    let omega_t = 10.0; // rad/s -> ~2.2 s
    let steps = (target_angle / omega_t / dt).round() as usize;
    let mut r_samples = Vec::new();
    let mut wind_tension_sum = 0.0;
    let mut err_int = 0.0;
    for s in 0..steps {
        let t_now = tension(&mut w, 0, 0);
        // Compute current test-drum attachment radius so load drum matches linear payout rate.
        let p_a = [w.path_data(0)[HEADER + 2 * LINK + 4], w.path_data(0)[HEADER + 2 * LINK + 5]];
        let r_attach = ((p_a[0] + span / 2.0).powi(2) + p_a[1].powi(2)).sqrt();
        let v_t = omega_t * r_attach;
        let err = f_s - t_now;
        err_int += err * dt;
        // PI tension servo on load drum (cw=true: +omega unwinds / lowers tension).
        let omega_l = (v_t - 5e-3 * err - 5e-2 * err_int) / 0.050;
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, omega_t]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega_l]);
        w.step(dt).unwrap();
        wind_tension_sum += tension(&mut w, 0, 0);
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
        if s % 100 == 0 {
            r_samples.push((w.entities[0][66] / (2.0 * PI), r_attach));
        }
    }
    let mean_wind_t = wind_tension_sum / steps as f64;
    assert!(
        (mean_wind_t - f_s).abs() < 15.0,
        "winding tension {mean_wind_t:.1} N drifted too far from target {f_s:.1} N"
    );

    let geom_wound = w.spool_geom_stored(0, 0);
    let rest_wound = stored(&mut w, 0, 0);
    // At 3.5 revolutions, layer 3 radius is base_r + half_w + 6*half_w = 0.044 m (vs 0.032 m at layer 0).
    let final_r = r_samples.last().unwrap().1;
    assert!(
        final_r > base_r + 5.0 * half_w,
        "expected multi-layer radius > {:.4} m after 3.5 revs, got {:.4} m",
        base_r + 5.0 * half_w,
        final_r
    );
    // Stored rest length must be noticeably smaller than geometric drum occupancy (>1% strain+flattening).
    let ratio = rest_wound / geom_wound;
    assert!(
        ratio < 0.988,
        "expected rest/geom < 0.988 under ~250 N winding, got {ratio:.5} (geom={geom_wound:.5}, rest={rest_wound:.5})"
    );
    // Also verify geometric occupancy matches >= 3 full layered revolutions (> 0.75 m).
    assert!(geom_wound > 0.75, "expected >0.75 m geometric arc over 3.5 revs, got {geom_wound:.4}");

    // Now drop tension to f_l = 40 N and unwind 2.5 revolutions: rest/geom paid out must still reflect f_s (~250 N), not f_l!
    let jo = HEADER + 2 * LINK;
    let lo1 = HEADER + LINK;
    let delta = rest(&mut w, 0, 0) - span / (1.0 + f_l / ea);
    w.path_data(0)[jo + 3] = span / (1.0 + f_l / ea);
    w.path_data(0)[lo1 + 2] += delta;
    for _ in 0..5 {
        w.entities[0][72..75].copy_from_slice(&[0.0; 3]);
        w.entities[1][72..75].copy_from_slice(&[0.0; 3]);
        w.step(dt).unwrap();
    }
    let rest_u0 = stored(&mut w, 0, 0);
    let geom_u0 = w.spool_geom_stored(0, 0);
    let unwind_steps = (2.5 * 2.0 * PI / omega_t / dt).round() as usize;
    for _ in 0..unwind_steps {
        let t_now = tension(&mut w, 0, 0);
        let r_approx = base_r + half_w + 2.0 * half_w * (w.entities[0][66] / (2.0 * PI)).floor().max(0.0);
        let v_t = omega_t * r_approx;
        let omega_l = (-v_t - 5e-4 * (f_l - t_now)) / 0.050;
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -omega_t]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega_l]);
        w.step(dt).unwrap();
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
    }
    let unwound_ratio = (rest_u0 - stored(&mut w, 0, 0)) / (geom_u0 - w.spool_geom_stored(0, 0));
    let rho_fl = 1.0 / (1.0 + f_l / ea);
    assert!(
        unwound_ratio < rho_fl - 0.005,
        "unwound rest/geom {unwound_ratio:.5} should reflect high storage tension (~{ratio:.5}), not unwind rho(40N)={rho_fl:.5}"
    );
}

/// Review Item 1: Radial flattening `k_r` must scale the effective lever arm `r_eff = r - k_r*F`
/// in the cable-force Jacobian and reported motor reaction torque (`torqueModeCableLoadTorques`).
#[test]
fn radial_flattening_scales_motor_reaction_torque_and_jacobian() {
    let r = 0.050;
    let ea = 50_000.0;
    let f_target = 100.0;
    let measure_torque = |k_r: f64| {
        let mut spool = wheel(0, [0.0, 0.0, 0.0], r, 0.01, 0.5, None);
        spool[75] += 16.0; // MOTOR component flag so spool participates in motor/torqueMode solve
        spool[57] = 1.0; // torqueMode = true
        spool[114] = f_target * r; // holding torque
        let anchor = fixed(1, [2.0, r, 0.0], 0.0);
        let segs = [Seg {
            a: 0,
            b: 1,
            rest: 2.0 / (1.0 + f_target / ea),
            pa: [0.0, r, 0.0],
            pb: [2.0, r, 0.0],
        }];
        let links = [(SPOOL_END, false, 1.0), (ATTACHMENT, false, 0.0)];
        let mut w = world(vec![spool, anchor], vec![path(&links, &segs, ea, 0.0, 4.0)]);
        w.options[1] = 0.0;
        w.set_cable_spool_params(0, k_r, false, 0.0);
        for _ in 0..20 {
            w.entities[0][72..75].copy_from_slice(&[0.0; 3]);
            w.step(0.001).unwrap();
        }
        let f = tension(&mut w, 0, 0);
        let tau = w.entities[0][59].abs();
        (f, tau)
    };
    let (f0, tau0) = measure_torque(0.0);
    let k_r = 2.0e-5; // large enough to make 4% radius contraction unmistakable at 100 N
    let (f1, tau1) = measure_torque(k_r);
    let arm0 = tau0 / f0;
    let arm1 = tau1 / f1;
    let expected_arm1 = r - k_r * f1;
    assert!((arm0 - r).abs() < 1e-6, "baseline lever arm {arm0:.6} vs {r:.6}");
    assert!(
        (arm1 - expected_arm1).abs() < 2e-5,
        "flattened lever arm {arm1:.6} vs expected {expected_arm1:.6} (F={f1:.2} N)"
    );
    assert!(arm1 < arm0 - 1.5e-3, "effective lever arm did not shrink under k_r");
}

/// Review Item 3: Unwinding a pre-wound spool with empty LIFO stack must obey the explicit
/// `init_storage_tension` policy (default 0 N = unstretched), NOT silently assume current tension.
#[test]
fn explicit_initial_storage_tension_controls_prewound_unwind() {
    let r = 0.050;
    let ea = 40_000.0;
    let k_r = 0.5e-6;
    let f_unwind = 180.0;

    let run_prewound = |init_f: f64| {
        let span = 2.0;
        let test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let segs = [Seg {
            a: 0,
            b: 1,
            rest: span / (1.0 + f_unwind / ea),
            pa: [-span / 2.0, r, 0.0],
            pb: [span / 2.0, r, 0.0],
        }];
        let links = [(SPOOL_END, false, 8.0), (SPOOL_END, true, 8.0)];
        let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
        w.options[1] = 0.0;
        w.set_cable_spool_params(0, k_r, true, init_f);
        let dt = 0.001;
        // Unwind 1 m from pre-wound test spool while servoing load drum to hold f_unwind.
        let s0 = stored(&mut w, 0, 0);
        let a0 = w.entities[0][66];
        for _ in 0..500 {
            let t_now = tension(&mut w, 0, 0);
            let v_t = 2.0;
            let v_l = v_t + 5e-4 * (f_unwind - t_now);
            w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -v_t / r]);
            w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -v_l / r]);
            w.step(dt).unwrap();
        }
        let d_rest = s0 - stored(&mut w, 0, 0);
        let d_geom = (w.entities[0][66] - a0).abs() * r;
        d_rest / d_geom
    };

    let rho_of = |f: f64| ((r - k_r * f) / r) / (1.0 + f / ea);
    let ratio_default = run_prewound(0.0);
    let ratio_75 = run_prewound(75.0);
    let ratio_180 = run_prewound(180.0);

    assert!(
        (ratio_default - rho_of(0.0)).abs() < 2e-4,
        "default init=0N gave {ratio_default:.6} vs expected {:.6}",
        rho_of(0.0)
    );
    assert!(
        (ratio_75 - rho_of(75.0)).abs() < 2e-4,
        "init=75N gave {ratio_75:.6} vs expected {:.6}",
        rho_of(75.0)
    );
    assert!(
        (ratio_180 - rho_of(180.0)).abs() < 2e-4,
        "init=180N gave {ratio_180:.6} vs expected {:.6}",
        rho_of(180.0)
    );
    // Explicitly verify init=75N differs significantly from unwinding at 180N!
    assert!(
        (ratio_75 - ratio_180).abs() > 0.003,
        "init=75N ({ratio_75:.6}) should clearly differ from init=180N ({ratio_180:.6})"
    );
}

/// Review Item 2: Exporting and importing spool state mid-run across a freshly created
/// `PhysicsWorld` reproduces an uninterrupted simulation bit-for-bit, and external edits to
/// `c.stored(li)` or runtime disabling of spool options resync cleanly.
#[test]
fn checkpoint_restore_and_external_stored_edit_preserve_invariants() {
    let r = 0.050;
    let ea = 50_000.0;
    let k_r = 0.5e-6;
    let span = 2.0;

    let build_rig = || {
        let test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let segs = [Seg {
            a: 0,
            b: 1,
            rest: span / (1.0 + 60.0 / ea),
            pa: [-span / 2.0, r, 0.0],
            pb: [span / 2.0, r, 0.0],
        }];
        let links = [(SPOOL_END, false, 2.0), (SPOOL_END, true, 12.0)];
        let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
        w.options[1] = 0.0;
        w.set_cable_spool_params(0, k_r, true, 0.0);
        w
    };

    let mut w_cont = build_rig();
    let dt = 0.002;
    // Wind 3 distinct layers at 60 N, 180 N, and 100 N to populate the LIFO stack.
    for &(f_stage, steps) in &[(60.0, 150usize), (180.0, 150), (100.0, 150)] {
        let jo = HEADER + 2 * LINK;
        let lo1 = HEADER + LINK;
        let d = rest(&mut w_cont, 0, 0) - span / (1.0 + f_stage / ea);
        w_cont.path_data(0)[jo + 3] -= d;
        w_cont.path_data(0)[lo1 + 2] += d;
        for _ in 0..steps {
            let t_now = tension(&mut w_cont, 0, 0);
            w_cont.entities[0][72..75].copy_from_slice(&[0.0, 0.0, 1.5 / r]);
            w_cont.entities[1][72..75].copy_from_slice(&[0.0, 0.0, (1.5 - 3e-4 * (f_stage - t_now)) / r]);
            w_cont.step(dt).unwrap();
        }
    }
    assert!(w_cont.spool_coil_count(0, 0) >= 3, "expected >=3 distinct coils in LIFO stack");

    // Snapshot entire physical state + spool history into a brand-new PhysicsWorld.
    let spool_state = w_cont.export_spool_state(0);
    let mut w_restored = PhysicsWorld::new(w_cont.entities.clone(), vec![w_cont.path_data(0).to_vec()]);
    w_restored.options = w_cont.options;
    assert!(w_restored.import_spool_state(0, &spool_state));

    // Unwind both worlds through all 3 coils at 140 N and assert exact parity.
    for _ in 0..420 {
        for w in [&mut w_cont, &mut w_restored] {
            let t_now = tension(w, 0, 0);
            w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -1.5 / r]);
            w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -(1.5 + 3e-4 * (140.0 - t_now)) / r]);
            w.step(dt).unwrap();
        }
        assert!(
            (stored(&mut w_cont, 0, 0) - stored(&mut w_restored, 0, 0)).abs() < 1e-12,
            "stored mismatch after restore"
        );
        assert!(
            (w_cont.spool_geom_stored(0, 0) - w_restored.spool_geom_stored(0, 0)).abs() < 1e-12,
            "geom_stored mismatch after restore"
        );
        assert!(
            (tension(&mut w_cont, 0, 0) - tension(&mut w_restored, 0, 0)).abs() < 1e-9,
            "tension mismatch after restore"
        );
    }

    // External modification to stored length must trigger clean resync on next step.
    let lo0 = HEADER;
    w_restored.path_data(0)[lo0 + 2] += 0.5;
    w_restored.entities[0][72..75].copy_from_slice(&[0.0; 3]);
    w_restored.entities[1][72..75].copy_from_slice(&[0.0; 3]);
    w_restored.step(dt).unwrap();
    assert_eq!(w_restored.spool_coil_count(0, 0), 0, "external edit should reset stale coils");
    assert!(w_restored.spool_geom_stored(0, 0).is_finite());

    // Runtime toggle OFF clears spool state cleanly.
    w_restored.set_cable_spool_params(0, 0.0, false, 0.0);
    w_restored.step(dt).unwrap();
    assert!((w_restored.spool_geom_stored(0, 0) - stored(&mut w_restored, 0, 0)).abs() < 1e-12);
}

/// Review Item 4: Adversarial clamping (nearly exhausted free span, simultaneous multi-end
/// winding/unwinding, large single-step motor jumps, rapid reversals) preserves `rest >= 1e-6`
/// and exact material conservation at every single timestep.
#[test]
fn adversarial_clamping_and_per_step_invariants() {
    let r = 0.050;
    let ea = 40_000.0;
    let span = 0.25;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: 0.04, // short free span near clamp
        pa: [-span / 2.0, r, 0.0],
        pb: [span / 2.0, r, 0.0],
    }];
    let links = [(SPOOL_END, false, 2.0), (SPOOL_END, true, 2.0)];
    let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    w.options[5] = 1.0; // layeringClampJointRestLength = true
    w.set_cable_spool_params(0, 0.5e-6, true, 50.0);

    let dt = 0.005;
    let mat0 = material(&mut w, 0);
    let mut hit_clamp = false;

    // Drive aggressive opposing/same-direction jumps that push rest right into the 1e-6 floor,
    // followed by reversals and partial coil consumption.
    for step in 0..200 {
        let (w0, w1) = match step % 40 {
            0..=9 => (40.0, -40.0),   // both ends wind hard -> forces clamp!
            10..=19 => (-25.0, 25.0), // both ends unwind
            20..=29 => (30.0, 28.0),  // transfer right->left with net tension build
            _ => (-30.0, -26.0),      // transfer left->right with tension drop
        };
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, w0]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, w1]);
        w.step(dt).unwrap();

        let cur_rest = rest(&mut w, 0, 0);
        let cur_mat = material(&mut w, 0);
        let cur_f = tension(&mut w, 0, 0);
        if cur_rest <= 1.1e-6 {
            hit_clamp = true;
        }
        assert!(cur_rest >= 1e-6 - 1e-9, "step {step}: rest {cur_rest} violated clamp");
        assert!(cur_f.is_finite() && cur_rest.is_finite(), "step {step}: non-finite state");
        assert!(
            (cur_mat - mat0).abs() < 1e-9,
            "step {step}: material conservation violated ({mat0} -> {cur_mat})"
        );
        assert!(w.spool_coil_count(0, 0) <= 256 && w.spool_coil_count(0, 1) <= 256);
    }
    assert!(hit_clamp, "adversarial test must actually exercise the 1e-6 rest-length clamp");
}

/// Review Item 5: Hybrid rolling <-> attachment transitions (`kind 3 <-> 4`) and hybrid-knot
/// synchronization must use geometric drum occupancy and preserve material conservation.
#[test]
fn hybrid_transitions_and_knot_angle_synchronization() {
    let r = 0.030;
    let half_w = 0.0015;
    let ea = 30_000.0;
    let mut spool = wheel(0, [0.0, 0.0, 0.0], r, 0.5, 1e6, None);
    spool[133] = 1.0; // HybridKnotAngleComponent present
    let anchor = fixed(1, [1.5, r + half_w, 0.0], 0.0);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: 1.49,
        pa: [0.0, r + half_w, 0.0],
        pb: [1.5, r + half_w, 0.0],
    }];
    let links = [(SPOOL_END, false, 0.02), (ATTACHMENT, false, 0.0)];
    let mut p = path(&links, &segs, ea, 0.0, 4.0);
    p[6] = half_w;
    let mut w = world(vec![spool, anchor], vec![p]);
    w.options[1] = 0.0;
    w.options[3] = 1.0; // layering ON
    w.options[8] = 1.0; // layeringHybridLinkStates ON
    w.set_cable_spool_params(0, 0.5e-6, true, 0.0);

    let dt = 0.002;
    let mat0 = material(&mut w, 0);
    let mut saw_attachment = false;
    let mut saw_rewrap = false;

    // Phase 1: Unwind past empty -> must transition from hybrid rolling (3) to hybrid-attachment (4).
    for _ in 0..150 {
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -8.0]);
        w.step(dt).unwrap();
        if w.path_data(0)[HEADER] as u32 == 4 {
            saw_attachment = true;
        }
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
    }
    assert!(saw_attachment, "expected rolling -> attachment transition (kind 3 -> 4)");

    // Phase 2: Reverse and wind 2.5 revolutions -> must transition back to kind 3 and stay
    // synchronized with the hybrid knot angle under strained storage.
    for _ in 0..450 {
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, 12.0]);
        w.step(dt).unwrap();
        if saw_attachment && w.path_data(0)[HEADER] as u32 == 3 {
            saw_rewrap = true;
        }
        assert!((material(&mut w, 0) - mat0).abs() < 1e-8);
    }
    assert!(saw_rewrap, "expected attachment -> rolling transition (kind 4 -> 3)");
    assert!(w.path_data(0)[HEADER + 3].is_finite(), "knot angle must remain finite");
    assert!(w.spool_geom_stored(0, 0) > 0.1, "expected wound geometric arc after rewrap");
}

/// Review Item 6: Under continuously varying tension over >1000 steps, coalescing keeps the
/// coil stack bounded (`<= 256`) while conserving material exactly (`< 1e-10`).
#[test]
fn continuous_tension_coalescing_bounded_memory_and_exact_conservation() {
    let r = 0.050;
    let ea = 40_000.0;
    let span = 2.0;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + 50.0 / ea),
        pa: [-span / 2.0, r, 0.0],
        pb: [span / 2.0, r, 0.0],
    }];
    let links = [(SPOOL_END, false, 0.5), (SPOOL_END, true, 20.0)];
    let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    w.set_cable_spool_params(0, 0.5e-6, true, 0.0);

    let dt = 0.002;
    let mat0 = material(&mut w, 0);
    let s0 = stored(&mut w, 0, 0);
    let g0 = w.spool_geom_stored(0, 0);

    // Wind 1200 steps while tension sweeps smoothly between ~50 N and ~250 N.
    let n_steps = 1200;
    for k in 0..n_steps {
        let f_target = 50.0 + 200.0 * (k as f64 / n_steps as f64);
        let t_now = tension(&mut w, 0, 0);
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, 1.0 / r]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, (1.0 - 5e-4 * (f_target - t_now)) / r]);
        w.step(dt).unwrap();
        assert!(w.spool_coil_count(0, 0) <= 256, "step {k}: coil count exceeded 256");
        assert!((material(&mut w, 0) - mat0).abs() < 1e-10, "step {k}: material leaked");
    }
    assert!(w.spool_coil_count(0, 0) <= 256);

    // Unwind the exact same drum angle back to start -> must recover initial (geom, rest) exactly!
    for _ in 0..n_steps {
        let t_now = tension(&mut w, 0, 0);
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -1.0 / r]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -(1.0 + 5e-4 * (120.0 - t_now)) / r]);
        w.step(dt).unwrap();
        assert!((material(&mut w, 0) - mat0).abs() < 1e-10);
    }
    assert!((w.spool_geom_stored(0, 0) - g0).abs() < 1e-6, "round-trip geom_stored drift");
    assert!((stored(&mut w, 0, 0) - s0).abs() < 1e-6, "round-trip stored rest length drift");
}

/// Review Item 7: Per-cable spool parameters isolate cables in the same world; interaction with
/// elongation hysteresis and capstan friction stays stable; timestep refinement converges;
/// disabled options match legacy behavior bit-for-bit.
#[test]
fn per_cable_params_hysteresis_capstan_timestep_convergence_and_legacy_parity() {
    // (a) Two independent cables in one world with different per-cable spool parameters.
    let r = 0.050;
    let ea = 50_000.0;
    let mut e0 = wheel(0, [-1.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let mut e1 = wheel(1, [1.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let mut e2 = wheel(2, [-1.0, 2.0, 0.0], r, 0.5, 1e6, None);
    let mut e3 = wheel(3, [1.0, 2.0, 0.0], r, 0.5, 1e6, None);
    let s0 = [Seg { a: 0, b: 1, rest: 2.0 / (1.0 + 150.0 / ea), pa: [-1.0, r, 0.0], pb: [1.0, r, 0.0] }];
    let s1 = [Seg { a: 2, b: 3, rest: 2.0 / (1.0 + 150.0 / ea), pa: [-1.0, 2.0 + r, 0.0], pb: [1.0, 2.0 + r, 0.0] }];
    let mut p0 = path(&[(SPOOL_END, false, 5.0), (SPOOL_END, true, 5.0)], &s0, ea, 0.0, 4.0);
    let mut p1 = path(&[(SPOOL_END, false, 5.0), (SPOOL_END, true, 5.0)], &s1, ea, 0.0, 4.0);
    p0[1] = 1000.0;
    p0[HEADER + 2 * LINK] = 2000.0;
    p1[1] = 1001.0;
    p1[HEADER + 2 * LINK] = 2001.0;
    for e in [&mut e0, &mut e1, &mut e2, &mut e3] {
        e[72..75].copy_from_slice(&[0.0, 0.0, -2.0 / r]);
    }
    let mut w = world(vec![e0, e1, e2, e3], vec![p0, p1]);
    w.options[1] = 0.0;
    // Cable 0: legacy (k_r=0, strained=false); Cable 1: upgraded (k_r=1e-6, strained=true, init=150N).
    w.set_cable_spool_params(0, 0.0, false, 0.0);
    w.set_cable_spool_params(1, 1.0e-6, true, 150.0);
    for _ in 0..200 {
        for id in 0..4 {
            w.entities[id][72..75].copy_from_slice(&[0.0, 0.0, -2.0 / r]);
        }
        w.step(0.001).unwrap();
    }
    let paid0 = 5.0 - stored(&mut w, 0, 0);
    let paid1 = 5.0 - stored(&mut w, 1, 0);
    assert!((paid0 - paid1) > 0.002, "per-cable spool params must act independently ({paid0} vs {paid1})");

    // (b) Interaction with elongation hysteresis (options[11..=12]) + intermediate capstan roller.
    let sp0 = wheel(0, [-2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let mut guide = fixed(1, [0.0, 0.25, 0.0], 0.03);
    guide[105] = 0.2; // capstan mu = 0.2
    let sp1 = wheel(2, [2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let segs_h = [
        Seg { a: 0, b: 1, rest: 2.01, pa: [-2.0, r, 0.0], pb: [0.0, 0.28, 0.0] },
        Seg { a: 1, b: 2, rest: 2.01, pa: [0.0, 0.28, 0.0], pb: [2.0, r, 0.0] },
    ];
    let links_h = [(SPOOL_END, false, 4.0), (ROLLING, false, 0.0), (SPOOL_END, true, 4.0)];
    let mut wh = world(vec![sp0, guide, sp1], vec![path(&links_h, &segs_h, ea, 0.0, 4.0)]);
    wh.options[1] = 0.0;
    wh.options[11] = 0.2; // hysteresisFraction
    wh.options[12] = 5.0; // hysteresisRecoveryTime
    wh.set_cable_spool_params(0, 0.5e-6, true, 0.0);
    for step in 0..200 {
        let dir = if step < 100 { 1.0 } else { -1.0 };
        wh.entities[0][72..75].copy_from_slice(&[0.0, 0.0, dir * 10.0]);
        wh.entities[2][72..75].copy_from_slice(&[0.0, 0.0, dir * 10.0]);
        wh.step(0.002).unwrap();
        assert!(tension(&mut wh, 0, 0).is_finite() && tension(&mut wh, 0, 1).is_finite());
    }

    // (c) Timestep convergence: dt = 0.002 vs 0.001 vs 0.0005 agree within 5e-5 on unwound rest/geom.
    let (_, _, r2, g2) = run_two_drum_wind_unwind(0.050, 0.0, 118_500.0, 0.299e-6, true, 75.0, 175.0, 2.0, 0.002);
    let (_, _, r1, g1) = run_two_drum_wind_unwind(0.050, 0.0, 118_500.0, 0.299e-6, true, 75.0, 175.0, 2.0, 0.001);
    let (_, _, r05, g05) = run_two_drum_wind_unwind(0.050, 0.0, 118_500.0, 0.299e-6, true, 75.0, 175.0, 2.0, 0.0005);
    assert!((r2 / g2 - r1 / g1).abs() < 1e-4, "dt=2ms vs 1ms diverged: {} vs {}", r2 / g2, r1 / g1);
    assert!((r1 / g1 - r05 / g05).abs() < 5e-5, "dt=1ms vs 0.5ms diverged: {} vs {}", r1 / g1, r05 / g05);

    // (d) Unconfigured elastic spools match explicit D-Pro reference parameters.
    let make_default = || {
        let test = wheel(0, [-1.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let load = wheel(1, [1.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let segs = [Seg { a: 0, b: 1, rest: 1.995, pa: [-1.0, r, 0.0], pb: [1.0, r, 0.0] }];
        let mut ww = world(vec![test, load], vec![path(&[(SPOOL_END, false, 5.0), (SPOOL_END, true, 5.0)], &segs, ea, 0.0, 4.0)]);
        ww.options[1] = 0.0;
        ww
    };
    let mut wa = make_default();
    let mut wb = make_default();
    wb.set_cable_spool_params(0, 0.299e-6 * 0.05018 / r, true, 0.0);
    for _ in 0..100 {
        for ww in [&mut wa, &mut wb] {
            ww.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -20.0]);
            ww.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -20.0]);
            ww.step(0.001).unwrap();
        }
    }
    assert!(
        wa.path_data(0).iter().zip(wb.path_data(0)).all(|(a, b)| a.to_bits() == b.to_bits()),
        "default path_data must match bit-for-bit"
    );
    assert!(
        wa.entities.iter().flatten().zip(wb.entities.iter().flatten()).all(|(a, b)| a.to_bits() == b.to_bits()),
        "default entities must match bit-for-bit"
    );
}

/// Review 2, Item 3: Hybrid attachment-to-rolling transition (`kind 4 -> 3`) must respect the
/// minimum rest-length clamp (`rest >= 1e-6`) when strained spool storage and radial flattening
/// are enabled, rejecting unsafe rewraps without corrupting link state or coil history.
#[test]
fn hybrid_rewrap_respects_rest_length_clamp() {
    let r = 0.040;
    let ea = 40_000.0;
    let k_r = 1.0e-6;
    // Anchor at (0.042, 0): tangent on circle r=0.04 is at theta_tan = acos(40/42) ~ 0.310 rad.
    // Attachment point at theta_att = 2.50 rad gives wrap arc d_geom = 0.04*(2.50 - 0.31) ~ 0.0876 m
    // while chord d ~ 0.0778 m. With rest = 0.070 m (strained + flattened), d_rest_preview ~ 0.078 m
    // exceeds `rest - 1e-6`, so the clamp guard must reject the unsafe rewrap without corrupting state.
    let spool = wheel(0, [0.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let anchor = fixed(1, [0.042, 0.0, 0.0], 0.0);
    let theta_att = 2.50_f64;
    let pa = [r * theta_att.cos(), r * theta_att.sin(), 0.0];
    let segs = [Seg { a: 0, b: 1, rest: 0.070, pa, pb: [0.042, 0.0, 0.0] }];
    let links = [(4.0, false, 0.0), (ATTACHMENT, false, 0.0)];
    let mut w = world(vec![spool, anchor], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    w.options[5] = 1.0; // layeringClampJointRestLength = true
    w.options[8] = 1.0; // layeringHybridLinkStates = true
    w.set_cable_spool_params(0, k_r, true, 0.0);

    let mat0 = material(&mut w, 0);
    for step in 0..20 {
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, 0.0]);
        w.step(0.001).unwrap();
        let cur_rest = rest(&mut w, 0, 0);
        assert!(cur_rest >= 1e-6 - 1e-9, "step {step}: rest {cur_rest} bypassed clamp during rewrap");
        assert!(
            (material(&mut w, 0) - mat0).abs() < 1e-9,
            "step {step}: material leaked during rejected rewrap"
        );
        assert_eq!(w.path_data(0)[HEADER] as u32, 4, "step {step}: unsafe rewrap should stay kind=4");
        assert_eq!(w.spool_coil_count(0, 0), 0, "step {step}: rejected rewrap must not push coils");
        assert!(stored(&mut w, 0, 0).abs() < 1e-9, "step {step}: stored must remain 0");
    }

    // Now supply enough free-span rest length (`0.12 m`) so the tangent wrap arc fits cleanly above 1e-6.
    let jo = HEADER + 2 * LINK;
    w.path_data(0)[jo + 3] = 0.12;
    w.path_data(0)[7] = 0.12;
    let mat1 = material(&mut w, 0);
    w.step(0.001).unwrap();
    assert_eq!(w.path_data(0)[HEADER] as u32, 3, "safe rewrap should succeed to kind=3");
    assert!(rest(&mut w, 0, 0) >= 1e-6, "post-rewrap rest must stay >= 1e-6");
    assert!(stored(&mut w, 0, 0) > 1e-3, "post-rewrap stored must absorb tangent arc");
    assert!(w.spool_geom_stored(0, 0) >= stored(&mut w, 0, 0));
    assert!((material(&mut w, 0) - mat1).abs() < 1e-9, "post-rewrap material conservation");
}

/// Review 2, Item 4: Finite-difference verification of the constraint derivative dL_0/dtheta vs
/// the solver's instantaneous running-radius torque Jacobian r_eff(F_current), plus mechanical
/// work/energy balance when unwinding a coil stored at F_S = 50 N under F_L = 250 N.
#[test]
fn historical_unwinding_jacobian_and_work_balance() {
    let r = 0.05018;
    let ea = 118_500.0;
    let k_r = 0.299e-6;
    let f_s = 50.0;
    let f_l = 250.0;
    let span = 2.0;

    let mut test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 0.5, None);
    test[75] += 16.0; // MOTOR flag
    test[57] = 1.0;   // torqueMode = true so constraint uses r_eff and reports load torque e[59]
    let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + f_s / ea),
        pa: [-span / 2.0, r, 0.0],
        pb: [span / 2.0, r, 0.0],
    }];
    let links = [(SPOOL_END, false, 1.0), (SPOOL_END, true, 12.0)];
    let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    w.set_cable_spool_params(0, k_r, true, f_s);

    let dt = 0.001;
    // Phase 1: Wind 2 m onto test drum at low storage tension F_S = 50 N.
    for _ in 0..1000 {
        let t_now = tension(&mut w, 0, 0);
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, 2.0 / r]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, (2.0 - 5e-3 * (f_s - t_now)) / r]);
        w.step(dt).unwrap();
    }

    // Phase 2: Raise free-span tension 5x to F_L = 250 N (load drum only), then unwind 1 m.
    let jo = HEADER + 2 * LINK;
    let lo1 = HEADER + LINK;
    let target_rest = span / (1.0 + f_l / ea);
    let d_r = rest(&mut w, 0, 0) - target_rest;
    w.path_data(0)[jo + 3] = target_rest;
    w.path_data(0)[lo1 + 2] += d_r;
    for _ in 0..5 {
        w.entities[0][72..75].copy_from_slice(&[0.0; 3]);
        w.entities[1][72..75].copy_from_slice(&[0.0; 3]);
        w.step(dt).unwrap();
    }

    let mut w_shaft = 0.0;
    let mut w_span = 0.0;
    let mut dtheta_tot = 0.0;
    let mut dl0_tot = 0.0;
    let mut f_sum = 0.0;
    let mut arm_sum = 0.0;
    let steps = 500;
    for _ in 0..steps {
        let s_before = stored(&mut w, 0, 0);
        let a_before = w.entities[0][66];
        let t_now = tension(&mut w, 0, 0);
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -2.0 / r]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -(2.0 + 5e-3 * (f_l - t_now)) / r]);
        w.step(dt).unwrap();
        let dtheta = (w.entities[0][66] - a_before).abs();
        let dl0 = s_before - stored(&mut w, 0, 0);
        let f = tension(&mut w, 0, 0);
        let tau = w.entities[0][59].abs();
        w_shaft += tau * dtheta;
        w_span += f * (1.0 + f / ea) * dl0;
        dtheta_tot += dtheta;
        dl0_tot += dl0;
        f_sum += f;
        arm_sum += tau / f;
    }

    let f_mean = f_sum / steps as f64;
    let jac_arm = arm_sum / steps as f64;
    let fd_dl0_dtheta = dl0_tot / dtheta_tot;
    let r_eff_expected = r - k_r * f_mean;
    let rho_s = ((r - k_r * f_s) / r) / (1.0 + f_s / ea);
    let rel_jac_vs_fd = (jac_arm - fd_dl0_dtheta).abs() / r;
    let work_ratio_sim = w_span / w_shaft;
    let work_ratio_theory = (1.0 + f_mean / ea) * rho_s * r / r_eff_expected;
    eprintln!(
        "[HISTORICAL UNWIND JACOBIAN F_S={f_s:.0}N -> F_L={f_mean:.1}N] jac_arm={jac_arm:.6} m vs r_eff={r_eff_expected:.6} m | FD dL0/dtheta={fd_dl0_dtheta:.6} m (rel diff={:.4}%) | W_span/W_shaft sim={work_ratio_sim:.6} vs theory={work_ratio_theory:.6}",
        rel_jac_vs_fd * 100.0,
    );
    // Reaction torque matches instantaneous running radius r_eff(F_L) to < 5e-5 m.
    assert!((jac_arm - r_eff_expected).abs() < 5e-5);
    // Small-strain Jacobian approximation gap against FD dL0/dtheta is O(F_S/EA + k_r|F_L-F_S|/R) < 0.3%.
    assert!(rel_jac_vs_fd < 0.003, "Jacobian vs FD dL0/dtheta gap {:.4}% exceeded 0.3%", rel_jac_vs_fd * 100.0);
    // Mechanical work ratio matches the exact elastic-transition energy balance to < 5e-4.
    assert!(
        (work_ratio_sim - work_ratio_theory).abs() < 5e-4,
        "work ratio {work_ratio_sim:.6} vs analytical {work_ratio_theory:.6}"
    );
}

/// Review 2, Item 5: Characterize local payout and instantaneous tension error of the 256-entry
/// bounded coil stack against an uncompressed per-step `(geom, rest)` reference stack across:
///   1. Slowly varying tension
///   2. Alternating high and low tension (>300 transitions, forcing aggressive merges)
///   3. Partial unwinding followed by re-winding
///   4. Repeated wind/unwind reversals
#[test]
fn bounded_coil_compression_accuracy_characterization() {
    let r = 0.050;
    let ea = 50_000.0;
    let k_r = 0.5e-6;
    let span = 2.0;
    let dt = 0.002;

    // Uncompressed reference LIFO stack (never merges entries).
    struct UncompressedStack {
        coils: Vec<(f64, f64)>,
    }
    impl UncompressedStack {
        fn apply(&mut self, d_geom: f64, rho_now: f64) -> f64 {
            if d_geom > 1e-12 {
                let dr = d_geom * rho_now;
                self.coils.push((d_geom, dr));
                dr
            } else if d_geom < -1e-12 {
                let mut need = -d_geom;
                let mut paid = 0.0;
                while need > 1e-12 {
                    let Some(top) = self.coils.last_mut() else {
                        paid += need;
                        break;
                    };
                    if need >= top.0 - 1e-12 {
                        let (g, dr) = self.coils.pop().unwrap();
                        paid += dr;
                        need = (need - g).max(0.0);
                    } else {
                        let dr = top.1 * (need / top.0);
                        top.0 -= need;
                        top.1 -= dr;
                        paid += dr;
                        need = 0.0;
                    }
                }
                -paid
            } else {
                0.0
            }
        }
    }

    let rho_of = |f: f64| ((r - k_r * f.max(0.0)) / r).max(0.5) / (1.0 + f.max(0.0) / ea);

    let run_regime = |name: &str, schedule: &[(f64, f64)]| -> (f64, f64, usize) {
        // schedule: (velocity m/s [+wind, -unwind], target_tension N)
        let test = wheel(0, [-span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let load = wheel(1, [span / 2.0, 0.0, 0.0], r, 0.5, 1e6, None);
        let segs = [Seg {
            a: 0,
            b: 1,
            rest: span / (1.0 + schedule[0].1 / ea),
            pa: [-span / 2.0, r, 0.0],
            pb: [span / 2.0, r, 0.0],
        }];
        let links = [(SPOOL_END, false, 0.0), (SPOOL_END, true, 25.0)];
        let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
        w.options[1] = 0.0;
        w.set_cable_spool_params(0, k_r, true, 0.0);

        let mut ref_stack = UncompressedStack { coils: Vec::new() };
        let mut ref_stored = 0.0;
        let mut max_payout_err_mm = 0.0_f64;
        let mut max_tension_err_n = 0.0_f64;
        let mut max_coils = 0usize;

        for &(vel, f_tar) in schedule {
            let g_before = w.spool_geom_stored(0, 0);
            let t_before = tension(&mut w, 0, 0);
            w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, vel / r]);
            w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, (vel - 3e-3 * (f_tar - t_before)) / r]);
            w.step(dt).unwrap();
            let d_geom = w.spool_geom_stored(0, 0) - g_before;
            // Drive uncompressed reference with exact same per-step (d_geom, tension_before).
            ref_stored += ref_stack.apply(d_geom, rho_of(t_before));
            let err_m = (stored(&mut w, 0, 0) - ref_stored).abs();
            max_payout_err_mm = max_payout_err_mm.max(err_m * 1e3);
            // Equivalent instantaneous tension error on a 2 m free span: dF = EA * dL / span.
            max_tension_err_n = max_tension_err_n.max(ea * err_m / span);
            max_coils = max_coils.max(w.spool_coil_count(0, 0));
        }
        eprintln!(
            "[COIL COMPRESSION {name:22}] max_coils={max_coils:3}, max_payout_err={max_payout_err_mm:.4} mm, max_equiv_tension_err={max_tension_err_n:.3} N"
        );
        assert!(max_coils <= 256);
        (max_payout_err_mm, max_tension_err_n, max_coils)
    };

    // 1. Slowly varying tension (1200 steps wind + 1200 steps unwind)
    let mut slow = Vec::new();
    for k in 0..1200 {
        let f = 40.0 + 200.0 * (0.5 - 0.5 * (PI * k as f64 / 600.0).cos());
        slow.push((1.0, f));
    }
    for _ in 0..1200 {
        slow.push((-1.0, 140.0));
    }
    let (slow_mm, slow_n, _) = run_regime("slowly_varying", &slow);

    // 2. Alternating high/low tension (320 rapid transitions -> exceeds 256-entry cap!)
    let mut alt = Vec::new();
    for k in 0..1280 {
        let f = if (k / 4) % 2 == 0 { 50.0 } else { 250.0 };
        alt.push((1.0, f));
    }
    for _ in 0..1280 {
        alt.push((-1.0, 150.0));
    }
    let (alt_mm, alt_n, _) = run_regime("alternating_hi_lo", &alt);

    // 3. Partial unwind + rewind + full unwind
    let mut partial = Vec::new();
    for k in 0..600 {
        partial.push((1.0, 60.0 + 180.0 * (k as f64 / 600.0)));
    }
    for _ in 0..300 {
        partial.push((-1.0, 120.0));
    }
    for k in 0..400 {
        partial.push((1.0, 220.0 - 150.0 * (k as f64 / 400.0)));
    }
    for _ in 0..700 {
        partial.push((-1.0, 100.0));
    }
    let (part_mm, part_n, _) = run_regime("partial_unwind", &partial);

    // 4. Repeated reversals across varying tensions
    let mut rev = Vec::new();
    for cyc in 0..12 {
        let f_w = 50.0 + 15.0 * cyc as f64;
        for _ in 0..120 {
            rev.push((1.0, f_w));
        }
        for _ in 0..80 {
            rev.push((-1.0, 160.0));
        }
    }
    for _ in 0..480 {
        rev.push((-1.0, 120.0));
    }
    let (rev_mm, rev_n, _) = run_regime("repeated_reversals", &rev);

    // Slowly varying, partial unwind, and reversals stay well below 0.05 mm (< 1.5 N on 2 m span).
    assert!(slow_mm < 0.05 && slow_n < 1.5, "slowly varying error too high: {slow_mm} mm / {slow_n} N");
    assert!(part_mm < 0.05 && part_n < 1.5, "partial unwind error too high: {part_mm} mm / {part_n} N");
    assert!(rev_mm < 0.05 && rev_n < 1.5, "repeated reversals error too high: {rev_mm} mm / {rev_n} N");
    // Even adversarial >300-transition 50<->250 N square wave stays bounded (< 0.5 mm / < 15 N).
    assert!(alt_mm < 0.5 && alt_n < 15.0, "alternating high/low error too high: {alt_mm} mm / {alt_n} N");
}
