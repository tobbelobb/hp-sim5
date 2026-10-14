use hp_sim5_cable_solver::{engine::PhysicsWorld, Entity};

fn entity(id: f64, x: f64) -> Entity {
    let mut e = [0.0; 160];
    e[0] = x;
    for i in [6, 13, 20, 24, 82, 89, 96, 100] {
        e[i] = 1.0;
    }
    e[50] = -1.0;
    e[51..55].fill(1.0);
    e[76..79].copy_from_slice(&[0.0, 0.0, 1.0]);
    e[106] = f64::NAN;
    e[115..121].fill(f64::NAN);
    e[127] = f64::NAN;
    e[128] = id;
    e
}
fn world(entities: Vec<Entity>, paths: Vec<Vec<f64>>) -> PhysicsWorld {
    let mut w = PhysicsWorld::new(entities, paths);
    w.options[3..6].fill(1.0);
    w.options[8] = 1.0;
    w.options[9] = -1.0;
    w.options[11] = 1e8;
    w.options[12..14].fill(1.0);
    w.options[18] = 1.0;
    w
}

#[test]
fn continuing_native_world_integrates_and_accepts_edits() {
    let mut e = entity(0.0, 0.0);
    e[75] = (1 | 4 | 128) as f64; // velocity, gravity, mass
    e[124] = 1.0;
    let mut w = world(vec![e], vec![]);
    w.options[2] = -9.81;
    for _ in 0..100 {
        w.step(0.002);
    }
    assert!((w.entities[0][72 - 1] + 1.962).abs() < 1e-11);
    assert!((w.entities[0][2] + 0.198162).abs() < 1e-11);
    w.entities[0][0] = 5.0;
    w.entities[0][69] = 2.0;
    w.step(0.002);
    assert!((w.entities[0][0] - 5.004).abs() < 1e-12);
}

#[test]
fn native_cable_solve_retains_lengths_and_forces() {
    let mut a = entity(0.0, 0.0);
    let mut b = entity(1.0, 3.0);
    for e in [&mut a, &mut b] {
        e[75] = 128.0;
        e[124] = 1.0;
    }
    let mut path = vec![
        1.0,
        3.0,
        f64::INFINITY,
        0.0,
        0.0,
        1.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        f64::NAN,
        0.0,
        0.0,
        0.0,
        0.0,
        f64::NAN,
        0.0,
    ];
    path.extend_from_slice(&[
        2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]);
    let mut w = world(vec![a, b], vec![path]);
    w.step(1.0);
    assert_eq!(w.entities[0][0], 1.0);
    assert_eq!(w.entities[1][0], 2.0);
    let data = w.path_data(0);
    assert_eq!(data[19 + 3], 1.0);
    assert_eq!(data[19 + 11], 1.0);
    assert_eq!(data[7], 1.0);
    for _ in 0..10 {
        w.step(1.0);
    }
    assert_eq!(w.entities[0][0], 1.0);
}

#[test]
fn native_position_and_torque_motors_use_the_same_core() {
    let mut e = entity(0.0, 0.0);
    e[75] = (2 | 8 | 16 | 64) as f64;
    e[25..34].copy_from_slice(&[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]);
    e[34..43].copy_from_slice(&[10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0]);
    e[109] = 0.5;
    e[110] = 50.0;
    e[111] = 0.01;
    e[112] = 600.0;
    e[113] = 1.0;
    e[107] = 0.1;
    e[67] = 1.0;
    let mut w = world(vec![e], vec![]);
    w.step(0.002);
    assert!((w.entities[0][66] - 0.1).abs() < 1e-12);
    w.entities[0][57] = 1.0;
    w.entities[0][114] = 0.2;
    w.step(0.002);
    assert!(w.entities[0][74] > 0.0);
    let mut independent = world(vec![entity(0.0, 0.0)], vec![]);
    independent.step(0.002);
    assert_eq!(independent.entities[0][66], 0.0);
}
