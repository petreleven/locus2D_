#include "../include/locus/plinks.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <math.h>
#include <raylib.h>

void DrawFilledPolygon(Vector2 *vertices, int vertexCount, Color color) {
  if (vertexCount < 3)
    return; // A polygon must have at least 3 vertices

  // Draw the triangles of the polygon
  for (int i = 1; i < vertexCount - 1; i++) {
    DrawTriangle(vertices[0], vertices[i], vertices[i + 1], color);
  }
}

ParticleCable::ParticleCable(real maxLength, real restitution, Particle &a,
                             Particle &b) {
  ParticleCable::maxLength = maxLength;
  ParticleCable::restitution = restitution;
  particles[0] = &a;
  particles[1] = &b;
}

real ParticleCable::currentLength() const {
  real length = (particles[1]->position - particles[0]->position).magnitude();
  return length;
}

unsigned ParticleCable::fillContact(ParticleContact *contact) const {
  real length = currentLength();
  if (length < maxLength) {
    return 0;
  }
  locus::Vector3 normal = particles[1]->position - particles[0]->position;
  normal.normalize();
  contact->p[0] = particles[0];
  contact->p[1] = particles[1];
  contact->penetration = length - maxLength;
  contact->contactNormal = normal;
  return 1;
}

void MassAggregate::addParticle(Particle &particle) {
  particles.push_back(particle);
  restPos.push_back(particle.position);
  if (particles.size() <= 1) {
    return;
  }
  ParticleSpring spring1(width, springK, damping);
  springForces.clear();
  springForces.push_back(spring1);
}

void MassAggregate::render() const {
  drawRectangle();
  for (size_t i = 0; i < particles.size(); i++) {
    int next = (i + 1) % particles.size();
    // DrawCircleV({particles[i].position.x, particles[i].position.y},
    //             particles[i].radius, color);
    /*DrawLineEx({particles[i].position.x, particles[i].position.y},
               {particles[next].position.x, particles[next].position.y}, 10,
               GREEN);*/
  }
  // DrawLineEx({particles[0].position.x, particles[0].position.y},
  //            {particles[2].position.x, particles[2].position.y}, 10, color);
  // DrawLineEx({particles[1].position.x, particles[1].position.y},
  //            {particles[3].position.x, particles[3].position.y}, 10, color);
}
void MassAggregate::update(real dt) {
  shapeMatch(dt);
  Particle *particlesTemp[2];

  for (size_t i = 0; i < particles.size(); i++) {
    particlesTemp[0] = &particles[i];
    particlesTemp[1] = &particles[(i + 1) % particles.size()];
    springForces[0].updateForce(particlesTemp, dt);
  }
  particlesTemp[0] = &particles[0];
  particlesTemp[1] = &particles[2];
  springForces[0].updateForce(particlesTemp, dt);
  particlesTemp[0] = &particles[1];
  particlesTemp[1] = &particles[3];
  springForces[0].updateForce(particlesTemp, dt);
  for (Particle &p : particles) {
    p.integrate(dt);
  }
}

void MassAggregate::setAcceleration(locus::Vector3 &acc) {
  for (Particle &p : particles) {
    p.setAcceleration(acc.x, acc.y, acc.z);
  }
}

static real skew2DProduct(locus::Vector3 a, locus::Vector3 b) {
  real ans = a.x * b.y - a.y * b.x;
  return ans;
}

void MassAggregate::shapeMatch(real dt) {
  locus::Vector3 originalCOM = locus::Vector3(0.f, 0.f, 0.f);
  locus::Vector3 currentCOM = locus::Vector3(0.f, 0.f, 0.f);
  for (size_t i = 0; i < particles.size(); i++) {
    originalCOM += restPos[i];
    currentCOM += particles[i].position;
  }
  originalCOM = locus::Vector3(originalCOM.x / restPos.size(),
                               originalCOM.y / restPos.size(), 0.f);
  currentCOM = locus::Vector3(currentCOM.x / particles.size(),
                              currentCOM.y / particles.size(), 0.f);
  std::vector<locus::Vector3> originalrelativeVectors;
  for (size_t i = 0; i < restPos.size(); i++) {
    locus::Vector3 rel = restPos[i] - originalCOM;
    originalrelativeVectors.push_back(rel);
  }
  real averageAngle = 0.0f;
  for (size_t i = 0; i < particles.size(); i++) {
    locus::Vector3 currentRelativePos = particles[i].position - currentCOM;
    real dot = currentRelativePos.dotProduct(originalrelativeVectors[i]);
    dot /= (currentRelativePos.magnitude() *
            originalrelativeVectors[i].magnitude());
    dot = std::max(-1.0f, dot);
    dot = std::min(1.0f, dot);
    real angle = acos(dot);
    real crossZ = skew2DProduct(currentRelativePos, originalrelativeVectors[i]);
    if (crossZ > 0) {
      angle = -angle;
    }
    averageAngle += angle;
  }
  averageAngle /= particles.size();
  std::vector<locus::Vector3> targetRelPositions;

  for (size_t i = 0; i < particles.size(); i++) {
    locus::Vector3 rotatedV =
        locus::Vector3(originalrelativeVectors[i].x * cos(averageAngle) -
                           originalrelativeVectors[i].y * sin(averageAngle),
                       originalrelativeVectors[i].x * sin(averageAngle) +
                           originalrelativeVectors[i].y * cos(averageAngle),
                       0.f);
    targetRelPositions.push_back(rotatedV);
  }
  const real stiffness = 20.f;
  for (size_t i = 0; i < particles.size(); i++) {
    locus::Vector3 finalTargpos = currentCOM + targetRelPositions[i];
    locus::Vector3 diff = finalTargpos - particles[i].position;
    auto f = diff * stiffness * dt;
    if (f.squareMagnitude() > 0.001f) {
      particles[i].velocity += f;
    }
    // DrawCircle(finalTargpos.x, finalTargpos.y, 20.f, RED);
  }
}

MassAggregateCircle::MassAggregateCircle(locus::Vector3 center, real radius) {
  const unsigned numPoints = 8;
  real z = 0.0f;
  for (size_t i = 0; i < numPoints; i++) {
    real theta = 2 * PI * i / numPoints;
    real x = center.x + radius * std::cos(theta);
    real y = center.y + radius * std::sin(theta);
    Particle p = Particle(locus::Vector3(x, y, z), locus::Vector3::Zero(), 0.8,
                          1, this->color);
    particles.push_back(p);
  }

  ParticleSpring spring1(2 * radius * std::sin(PI / numPoints), springK,
                         damping);
  ParticleSpring spring2(radius * 2, springK * 4, damping);
  springForces.push_back(spring1);
  springForces.push_back(spring2);
}

void MassAggregateCircle::render() const {
  for (size_t i = 0; i < particles.size(); i++) {
    int next = (i + 1) % particles.size();
    DrawCircleV({particles[i].position.x, particles[i].position.y},
                particles[i].radius, color);
    DrawLineEx({particles[i].position.x, particles[i].position.y},
               {particles[next].position.x, particles[next].position.y}, 11,
               color);
  }
}

void MassAggregateCircle::update(real dt) {
  for (Particle &p : particles) {
    p.integrate(dt);
  }
  Particle *particlesTemp[2];
  for (size_t i = 0; i < particles.size(); i++) {
    particlesTemp[0] = &particles[i];
    particlesTemp[1] = &particles[(i + 1) % particles.size()];
    springForces[0].updateForce(particlesTemp, dt);
  }
  // Diagonal springs
  for (size_t i = 0; i < 4; i++) {
    particlesTemp[0] = &particles[i];
    particlesTemp[1] = &particles[(i + 4) % particles.size()];
    springForces[1].updateForce(particlesTemp, dt);
  }
}

void MassAggregateCircle::setAcceleration(locus::Vector3 &c) {
  for (Particle &p : particles) {
    p.setAcceleration(c.x, c.y, c.z);
  }
}
