// Node-API host for the same Rust C ABI used by Wasm. No physics lives here.
#define NAPI_VERSION 8
#include <node_api.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define PATH_HEADER 16
#define PATH_LINK 5
#define PATH_JOINT 16

typedef struct PhysicsWorld PhysicsWorld;
extern PhysicsWorld *engine_create(const double *, size_t, const double *, size_t);
extern void engine_destroy(PhysicsWorld *);
extern void engine_step(PhysicsWorld *, double);
extern double *engine_entities(PhysicsWorld *);
extern double *engine_options(PhysicsWorld *);
extern double *engine_path(PhysicsWorld *, size_t);
extern size_t engine_path_len(PhysicsWorld *, size_t);
extern uint32_t engine_topology_version(const PhysicsWorld *);
extern size_t engine_options_len(void);

typedef struct {
    PhysicsWorld *world;
    size_t refs, entities, paths;
    int closed;
    // Weak references: views retain the allocation, never the JS world object.
    napi_ref *buffers;
} Host;

#define CHECK(call) do { if ((call) != napi_ok) { \
    napi_throw_error(env, NULL, "Native physics Node-API call failed"); return NULL; \
} } while (0)

static void release(Host *host) {
    if (--host->refs) return;
    engine_destroy(host->world);
    free(host->buffers);
    free(host);
}
static void finalize_buffer(napi_env env, void *data, void *hint) {
    (void)env; (void)data;
    release(hint);
}
static void finalize_host(napi_env env, void *data, void *hint) {
    (void)hint;
    Host *host = data;
    for (size_t i = 0; i < host->paths + 2; i++)
        if (host->buffers[i]) napi_delete_reference(env, host->buffers[i]);
    release(host);
}
static napi_value buffer_view(napi_env env, Host *host, size_t slot, double *data, size_t len) {
    napi_value buffer, array;
    if (host->buffers[slot]) {
        CHECK(napi_get_reference_value(env, host->buffers[slot], &buffer));
        if (buffer) {
            CHECK(napi_create_typedarray(env, napi_float64_array, len, buffer, 0, &array));
            return array;
        }
        CHECK(napi_delete_reference(env, host->buffers[slot]));
        host->buffers[slot] = NULL;
    }
    CHECK(napi_create_external_arraybuffer(env, data, len * sizeof(double), finalize_buffer, host, &buffer));
    host->refs++;
    CHECK(napi_create_reference(env, buffer, 0, &host->buffers[slot]));
    CHECK(napi_create_typedarray(env, napi_float64_array, len, buffer, 0, &array));
    return array;
}
static int detach(napi_env env, Host *host, size_t slot) {
    if (!host->buffers[slot]) return 1;
    napi_value buffer;
    bool detached;
    if (napi_get_reference_value(env, host->buffers[slot], &buffer) != napi_ok) return 0;
    if (buffer && (napi_is_detached_arraybuffer(env, buffer, &detached) != napi_ok ||
        (!detached && napi_detach_arraybuffer(env, buffer) != napi_ok))) return 0;
    if (napi_delete_reference(env, host->buffers[slot]) != napi_ok) return 0;
    host->buffers[slot] = NULL;
    return 1;
}
static Host *receiver(napi_env env, napi_callback_info info, size_t *argc, napi_value *args) {
    napi_value self;
    Host *host = NULL;
    if (napi_get_cb_info(env, info, argc, args, &self, NULL) != napi_ok ||
        napi_unwrap(env, self, (void **)&host) != napi_ok || !host) {
        napi_throw_type_error(env, NULL, "Expected a native physics world"); return NULL;
    }
    if (host->closed) { napi_throw_error(env, NULL, "Native physics world is disposed"); return NULL; }
    return host;
}
static napi_value entities(napi_env env, napi_callback_info info) {
    size_t argc = 0; Host *host = receiver(env, info, &argc, NULL);
    return host ? buffer_view(env, host, 0, engine_entities(host->world), host->entities * 160) : NULL;
}
static napi_value options(napi_env env, napi_callback_info info) {
    size_t argc = 0; Host *host = receiver(env, info, &argc, NULL);
    return host ? buffer_view(env, host, 1, engine_options(host->world), engine_options_len()) : NULL;
}
static napi_value path(napi_env env, napi_callback_info info) {
    size_t argc = 1; napi_value args[1];
    Host *host = receiver(env, info, &argc, args); if (!host) return NULL;
    double index;
    if (argc != 1 || napi_get_value_double(env, args[0], &index) != napi_ok ||
        !isfinite(index) || index < 0 || index >= host->paths || index != floor(index)) {
        napi_throw_range_error(env, NULL, "Invalid native path index"); return NULL;
    }
    return buffer_view(env, host, (size_t)index + 2, engine_path(host->world, (size_t)index),
        engine_path_len(host->world, (size_t)index));
}
static napi_value step(napi_env env, napi_callback_info info) {
    size_t argc = 1; napi_value args[1], result;
    Host *host = receiver(env, info, &argc, args); if (!host) return NULL;
    double dt;
    if (argc != 1 || napi_get_value_double(env, args[0], &dt) != napi_ok || !isfinite(dt) || dt < 0) {
        napi_throw_range_error(env, NULL, "Native timestep must be nonnegative and finite"); return NULL;
    }
    uint32_t before = engine_topology_version(host->world);
    engine_step(host->world, dt);
    uint32_t after = engine_topology_version(host->world);
    // Rust can reallocate path storage during split/merge. JS cannot run during
    // this synchronous call; detach old views before returning to JS.
    if (before != after) for (size_t i = 2; i < host->paths + 2; i++) {
        if (!detach(env, host, i)) {
            napi_fatal_error("native physics", NAPI_AUTO_LENGTH, "Cannot detach obsolete path view", NAPI_AUTO_LENGTH);
        }
    }
    CHECK(napi_create_uint32(env, after, &result)); return result;
}
static napi_value dispose(napi_env env, napi_callback_info info) {
    size_t argc = 0; Host *host = receiver(env, info, &argc, NULL); if (!host) return NULL;
    for (size_t i = 0; i < host->paths + 2; i++) if (!detach(env, host, i)) {
        napi_throw_error(env, NULL, "Cannot detach native physics view"); return NULL;
    }
    host->closed = 1;
    napi_value result; CHECK(napi_get_undefined(env, &result)); return result;
}
static int doubles(napi_env env, napi_value value, double **data, size_t *len) {
    napi_typedarray_type type;
    if (napi_get_typedarray_info(env, value, &type, len, (void **)data, NULL, NULL) != napi_ok ||
        type != napi_float64_array) return 0;
    return 1;
}
static int index_valid(double value, size_t count) {
    return isfinite(value) && value >= 0 && value < count && value == floor(value);
}
static napi_value create(napi_env env, napi_callback_info info) {
    size_t argc = 3, entity_len, path_len;
    napi_value args[3], object;
    double *entity_data, *path_data, path_count;
    CHECK(napi_get_cb_info(env, info, &argc, args, NULL, NULL));
    if (argc != 3 || !doubles(env, args[0], &entity_data, &entity_len) || entity_len % 160 ||
        !doubles(env, args[1], &path_data, &path_len) ||
        napi_get_value_double(env, args[2], &path_count) != napi_ok || !isfinite(path_count) ||
        path_count < 0 || path_count > path_len || path_count != floor(path_count)) {
        napi_throw_type_error(env, NULL, "Expected prepared Float64Array entity/path records and path count"); return NULL;
    }
    size_t count = entity_len / 160, cursor = 0;
    for (size_t i = 0; i < count; i++) {
        double parent = entity_data[i * 160 + 50];
        if (parent != -1 && (!index_valid(parent, count) || parent == i)) {
            napi_throw_range_error(env, NULL, "Invalid prepared parent index"); return NULL;
        }
    }
    for (size_t i = 0; i < count; i++) {
        size_t ancestor = i, depth = 0;
        while (entity_data[ancestor * 160 + 50] >= 0) {
            ancestor = (size_t)entity_data[ancestor * 160 + 50];
            if (ancestor == i || ++depth >= count) {
                napi_throw_range_error(env, NULL, "Cyclic prepared parent indices"); return NULL;
            }
        }
    }
    for (size_t p = 0; p < (size_t)path_count; p++) {
        if (cursor >= path_len || !index_valid(path_data[cursor], path_len + 1)) goto invalid;
        size_t len = (size_t)path_data[cursor++];
        if (len < PATH_HEADER + PATH_LINK || len > path_len - cursor) goto invalid;
        double n = path_data[cursor];
        if (!index_valid(n, len) || len != PATH_HEADER + ((size_t)n + 1) * PATH_LINK + (size_t)n * PATH_JOINT) goto invalid;
        for (size_t i = 0; i <= (size_t)n; i++)
            if (!index_valid(path_data[cursor + PATH_HEADER + i * PATH_LINK], 6)) goto invalid;
        for (size_t i = 0; i < (size_t)n; i++) {
            size_t joint = cursor + PATH_HEADER + ((size_t)n + 1) * PATH_LINK + i * PATH_JOINT;
            if (!index_valid(path_data[joint + 1], count) || !index_valid(path_data[joint + 2], count)) goto invalid;
        }
        cursor += len;
    }
    if (cursor != path_len) goto invalid;
    Host *host = calloc(1, sizeof(Host));
    if (!host) { napi_throw_error(env, NULL, "Cannot allocate native physics host"); return NULL; }
    host->buffers = calloc((size_t)path_count + 2, sizeof(napi_ref));
    if (!host->buffers) { free(host); napi_throw_error(env, NULL, "Cannot allocate native physics views"); return NULL; }
    static const double empty = 0; // Rust slices require aligned non-null pointers even at length zero.
    host->world = engine_create(entity_data ? entity_data : &empty, count, path_data ? path_data : &empty, (size_t)path_count);
    host->refs = 1; host->entities = count; host->paths = (size_t)path_count;
    CHECK(napi_create_object(env, &object));
    CHECK(napi_wrap(env, object, host, finalize_host, NULL, NULL));
    napi_property_descriptor properties[] = {
        {"entities", NULL, entities, NULL, NULL, NULL, napi_default, NULL},
        {"options", NULL, options, NULL, NULL, NULL, napi_default, NULL},
        {"path", NULL, path, NULL, NULL, NULL, napi_default, NULL},
        {"step", NULL, step, NULL, NULL, NULL, napi_default, NULL},
        {"dispose", NULL, dispose, NULL, NULL, NULL, napi_default, NULL},
    };
    CHECK(napi_define_properties(env, object, 5, properties)); return object;
invalid:
    napi_throw_range_error(env, NULL, "Invalid prepared native cable path"); return NULL;
}
NAPI_MODULE_INIT() {
    napi_property_descriptor property = {"create", NULL, create, NULL, NULL, NULL, napi_default, NULL};
    CHECK(napi_define_properties(env, exports, 1, &property)); return exports;
}
