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
/// # Safety
/// `world` must be live and exclusively owned. Views must not be used during step.
#[no_mangle]
pub unsafe extern "C" fn engine_step(world: *mut PhysicsWorld, dt: f64) {
    (*world).step(dt);
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
