#pragma once

#include "core.h"
#include "forcegenerator.h"
#include "particle.h"
#include "pcontacts.h"
#include "precision.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <raylib.h>
#include <tuple>
#include <vector>
class ParticleContact;
class ParticleLink {
private:
protected:
  virtual real currentLength() const = 0;

public:
  Particle *particles[2];
  virtual unsigned fillContact(ParticleContact *contact) const = 0;
};

class ParticleCable : public ParticleLink {
private:
  real maxLength;
  real currentLength() const override;
  real restitution;

public:
  ParticleCable(real maxLength, real restitution, Particle &a, Particle &b);
  unsigned fillContact(ParticleContact *contact) const override;
};

class MassAggregate {
private:
  void shapeMatch(real dt);
  void drawRectangle() const {

    if (particles.size() < 4)
      return; // Need at least 4 particles to form a rectangle

    // Project 3D positions to 2D (assuming z is depth)
    std::vector<Vector2> points2D;
    for (const auto &particle : particles) {
      points2D.push_back({particle.position.x, particle.position.y});
    }

    // Calculate center
    Vector2 center = {0, 0};
    for (const auto &point : points2D) {
      center = {center.x + point.x, center.y + point.y};
    }
    center = {center.x / points2D.size(), center.y / points2D.size()};

    // Sort points based on their angle from the center
    std::sort(points2D.begin(), points2D.end(),
              [center](const Vector2 &a, const Vector2 &b) {
                return atan2(a.y - center.y, a.x - center.x) >
                       atan2(b.y - center.y, b.x - center.x);
              });
    if (points2D.size() >= 3) {
      for (size_t i = 0; i < points2D.size() - 2; i++) {
        DrawTriangle(points2D[0], points2D[i + 1], points2D[i + 2], color);
      }
    }
  }

public:
  std::vector<Particle> particles;
  std::vector<locus::Vector3> restPos;
  Color color = BLACK;
  virtual void update(real dt);
  virtual void addParticle(Particle &particle);
  virtual void render() const;
  virtual void setAcceleration(locus::Vector3 &acc);
};

class MassAggregateCircle : public MassAggregate {

public:
  MassAggregateCircle(locus::Vector3 center, real radius);
  void render() const override;
  void update(real dt) override;
  void setAcceleration(locus::Vector3 &acc) override;
};
