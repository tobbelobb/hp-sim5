#pragma once

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    Vec3 operator+(const Vec3& other) const {
        return Vec3{x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3{x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(float s) const {
        return Vec3{x * s, y * s, z * s};
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

struct StaticForcesConfig {
    bool ignoreGravity = false;
    bool ignorePretension = false;
    float massKg = 0.0f;
    float g = 9.81f;
    float lambda = 1e-3f;
    float tol = 1e-3f; // Millinewton precision on pretension solution
    float stepDamp = 0.75f;
    int maxItersTarget = 100;
    float* Tmax;
    float* Tmin;
};

struct StaticForcesResult {
    float* tensions = nullptr;
    Vec3 achievedForce = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 requestedForce = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 residual = Vec3(0.0f, 0.0f, 0.0f);
    float supportedGravityFrac = 0.0f;
};

void StaticForcesEx(const Vec3* anchors,
                    int N,
                    const Vec3& mover,
                    const StaticForcesConfig& cfg,
                    StaticForcesResult& out);

void StaticForcesEx_qp(const Vec3* anchors,
                       int N,
                       const Vec3& mover,
                       const StaticForcesConfig& cfg,
                       StaticForcesResult& out);
