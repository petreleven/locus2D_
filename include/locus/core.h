#pragma once
#include "precision.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>

static std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));
namespace locus {

class Vector3 {

public:
  real x;
  real y;
  real z;
  static const locus::Vector3 GRAVITY;

private:
  real pad;

public:
  Vector3() : x(0), y(0), z(0) {}
  Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}
  void invert() {
    x = -x;
    y = -y;
    z = -z;
  }
  static real locusRandom(real max, real min) {
    std::uniform_real_distribution<real> distribution(min, max);
    return distribution(rng);
  }
  real real_sqrt(real squareMag) { return std::sqrt(squareMag); }

  real magnitude() { return real_sqrt(x * x + y * y + z * z); }
  real squareMagnitude() const { return x * x + y * y + z * z; }
  void normalize() {
    real _magnitude = magnitude();
    if (_magnitude > 0) {
      x = x * 1 / _magnitude;
      y = y * 1 / _magnitude;
      z = z * 1 / _magnitude;
    }
  }
  // SCALAR MULTIPLICATION of this vector
  void operator*=(const real value) {
    x *= value;
    y *= value;
    z *= value;
  }
  // Return a scalar Copy
  Vector3 operator*(const real value) const {
    return Vector3(x * value, y * value, z * value);
  }
  // ADD TO THIS
  void operator+=(const Vector3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  // Return and Vector added to this
  Vector3 operator+(const Vector3 &v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
  }
  // SUBSTRACT FROM THIS
  void operator-=(const Vector3 &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }
  // SUUBSTRACT AND RETURN NEW Vec3
  Vector3 operator-(const Vector3 &v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }
  // add scaled Vector
  void addScaledVector(const Vector3 &v, real scale) {
    Vector3 vToAdd = v * scale;
    (*this) += vToAdd;
  }
  // COMPONENT PRODUCT
  void componentProduct(const Vector3 &v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
  }
  Vector3 componentProduct(const Vector3 &v) const {
    return Vector3(x * v.x, y * v.y, z * v.z);
  }
  real dotProduct(const Vector3 &v) const {
    return (x * v.x + y * v.y + z * v.z);
  }
  // CROSS PROD OR VECTOR PRODUCT
  Vector3 crossProduct(const Vector3 &v) const {
    real new_x = y * v.z - z * v.y;
    real new_y = z * v.x - x * v.z;
    real new_z = x * v.y - y * v.x;
    return Vector3(new_x, new_y, new_z);
  }
  void operator%=(const Vector3 &v) { (*this) = crossProduct(v); }
  static Vector3 randomVector(Vector3 min, Vector3 max) {
    return Vector3(locusRandom(max.x, min.x), locusRandom(max.y, min.y),
                   locusRandom(max.z, min.z));
  }
  Vector3 &operator=(const Vector3 &v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
};
} // namespace locus
