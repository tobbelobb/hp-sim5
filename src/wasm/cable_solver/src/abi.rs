//! Thin ABI only. Native consumers use engine::PhysicsWorld directly.
use crate::{
    engine::{PhysicsWorld, OPTIONS},
    Entity,
};

// Pointers and lengths come from this module's allocator and scene adapter.
// A world is exclusively borrowed for each call, and is never shared by worlds.
/// # Safety
/// Entity and path buffers must contain valid prepared records and indices.
#[no_mangle]
pub unsafe extern "C" fn engine_create(
    entities: *const Entity,
    count: usize,
    paths: *const f64,
    path_count: usize,
) -> *mut PhysicsWorld {
    let entities = std::slice::from_raw_parts(entities, count).to_vec();
    let mut prepared = Vec::with_capacity(path_count);
    let mut ptr = paths;
    for _ in 0..path_count {
        let len = *ptr as usize;
        ptr = ptr.add(1);
        prepared.push(std::slice::from_raw_parts(ptr, len).to_vec());
        ptr = ptr.add(len);
    }
    Box::into_raw(Box::new(PhysicsWorld::new(entities, prepared)))
}
/// # Safety
/// `world` must be a live engine_create allocation, exclusively owned by caller.
#[no_mangle]
pub unsafe extern "C" fn engine_destroy(world: *mut PhysicsWorld) {
    drop(Box::from_raw(world));
}
/// Returns zero on success or the one-based row of a non-finite position.
/// # Safety
/// `world` must be live and exclusively owned. Views must not be used during step.
#[no_mangle]
pub unsafe extern "C" fn engine_step(world: *mut PhysicsWorld, dt: f64) -> usize {
    match (*world).step(dt) {
        Ok(()) => 0,
        Err(error) => error.row + 1,
    }
}
/// # Safety
/// `world` must be live. The view lasts until engine_destroy.
#[no_mangle]
pub unsafe extern "C" fn engine_entities(world: *mut PhysicsWorld) -> *mut Entity {
    (*world).entities.as_mut_ptr()
}
/// # Safety
/// `world` must be live.
#[no_mangle]
pub unsafe extern "C" fn engine_entity_count(world: *const PhysicsWorld) -> usize {
    (*world).entities.len()
}
/// # Safety
/// `world` must be live. The view lasts until engine_destroy.
#[no_mangle]
pub unsafe extern "C" fn engine_options(world: *mut PhysicsWorld) -> *mut f64 {
    (*world).options.as_mut_ptr()
}
/// # Safety
/// `world` must be live; `index` is in 0..path_count. Refresh after topology edits.
#[no_mangle]
pub unsafe extern "C" fn engine_path(world: *mut PhysicsWorld, index: usize) -> *mut f64 {
    (*world).path_data(index).as_mut_ptr()
}
/// # Safety
/// `world` must be live; `index` is in 0..path_count.
#[no_mangle]
pub unsafe extern "C" fn engine_path_len(world: *mut PhysicsWorld, index: usize) -> usize {
    (*world).path_data(index).len()
}
/// # Safety
/// `world` must be live.
#[no_mangle]
pub unsafe extern "C" fn engine_topology_version(world: *const PhysicsWorld) -> u32 {
    (*world).topology_version
}
#[no_mangle]
pub extern "C" fn engine_options_len() -> usize {
    OPTIONS
}
/// # Safety
/// `world` must be live; `index` is in `0..path_count`.
#[no_mangle]
pub unsafe extern "C" fn engine_set_cable_spool_params(
    world: *mut PhysicsWorld,
    index: usize,
    radial_flattening: f64,
    strained_mode: f64,
    init_storage_tension: f64,
) {
    (*world).set_cable_spool_params_raw(index, radial_flattening, strained_mode, init_storage_tension);
}
/// # Safety
/// `world` must be live; `index` is in `0..path_count`.
#[no_mangle]
pub unsafe extern "C" fn engine_spool_state_len(world: *const PhysicsWorld, index: usize) -> usize {
    (*world).export_spool_state(index).len()
}
/// # Safety
/// `world` must be live and `out` must point to at least `cap` writable f64s.
#[no_mangle]
pub unsafe extern "C" fn engine_export_spool_state(
    world: *const PhysicsWorld,
    index: usize,
    out: *mut f64,
    cap: usize,
) -> usize {
    let state = (*world).export_spool_state(index);
    let n = state.len().min(cap);
    if n > 0 && !out.is_null() {
        std::ptr::copy_nonoverlapping(state.as_ptr(), out, n);
    }
    n
}
/// # Safety
/// `world` must be live and `data` must point to `len` readable f64s when `len > 0`.
#[no_mangle]
pub unsafe extern "C" fn engine_import_spool_state(
    world: *mut PhysicsWorld,
    index: usize,
    data: *const f64,
    len: usize,
) -> u32 {
    if len == 0 || data.is_null() {
        return 0;
    }
    let slice = std::slice::from_raw_parts(data, len);
    u32::from((*world).import_spool_state(index, slice))
}
