#include "../include/locus/pcontacts.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <raylib.h>
#include <stdexcept>

void ParticleContact::resolve(locus::real dt) {

  resolveVelocity(dt);
  resolvePenetration(dt);
}

locus::real ParticleContact::calculateSeparatingV() {
  // Vseparating_b4_contact = (v0 - v1) . normalContact;
  locus::Vector3 relativeVelocity = p[0]->velocity;
  if (p[1]) {
    relativeVelocity -= p[1]->velocity;
  }
  locus::real vs = relativeVelocity.dotProduct(contactNormal);
  return vs;
}

void ParticleContact::resolveVelocity(locus::real dt) {
  locus::real vs_b4_contact = calculateSeparatingV();

  // no need to resolve collision
  if (vs_b4_contact > 0) {
    return;
  }
  locus::real vs_after_contact = -vs_b4_contact * restitution;
  // RESTING
  locus::Vector3 accCausedVelocity = p[0]->acceleration;
  if (p[1]) {
    accCausedVelocity -= p[1]->acceleration;
  }
  real accCausedSepVelocity = accCausedVelocity.dotProduct(contactNormal) * dt;
  // vel is due to v build up
  if (fabs(accCausedSepVelocity) < 0.001f) {
    vs_after_contact += accCausedSepVelocity * restitution;
    if (vs_after_contact < 0.001f) {
      vs_after_contact = 0.f;
    }
  }
  locus::real deltaVs = vs_after_contact - vs_b4_contact;
  locus::real totalInverseMass = p[0]->getInverseMass();
  if (p[1]) {
    totalInverseMass += p[1]->getInverseMass();
  }
  if (totalInverseMass <= 0) {
    return;
  }
  locus::real impulse = deltaVs / totalInverseMass;
  locus::Vector3 impulseV = contactNormal * impulse;

  // p0_vel = p0_vel + impulse / p0_mass
  p[0]->velocity += impulseV * p[0]->getInverseMass();
  // p1_vel = p1_vel - impulse / p1_mass
  if (p[1]) {

    p[1]->velocity -= impulseV * p[1]->getInverseMass();
  }
}

void ParticleContact::resolvePenetration(real dt) {
  if (penetration <= 0.001f)
    return;
  real totalInverseMass = p[0]->getInverseMass();
  if (p[1]) {
    totalInverseMass += p[1]->getInverseMass();
  }
  if (totalInverseMass <= 0.0f) {
    return;
  }
  locus::Vector3 movementPen = contactNormal * (penetration / totalInverseMass);
  locus::Vector3 move_0 = movementPen * p[0]->getInverseMass();
  move[0] = move_0;
  p[0]->position += move[0];
  if (p[1]) {
    locus::Vector3 move_1 = movementPen * p[1]->getInverseMass() * -1;
    move[1] = move_1;
    p[1]->position += move[1];
  }
}

void ParticleEdgeContact::resolve(real dt) {
  Particle &edgeParticles0 = shape->particles[edgeParticleIndices[0]];
  Particle &edgeParticles1 = shape->particles[edgeParticleIndices[1]];
  locus::Vector3 edgeToEdge = edgeParticles1.position - edgeParticles0.position;
  locus::Vector3 edgeToEdgeNormalized = edgeToEdge;
  edgeToEdgeNormalized.normalize();
  locus::Vector3 collidingParticleToEdge =
      particleColliding->position - edgeParticles0.position;
  real projection = collidingParticleToEdge.dotProduct(edgeToEdgeNormalized);
  real edgeRatio = projection / edgeToEdge.magnitude();
  // compute impulse
  locus::Vector3 virtualEdgeVelocity =
      edgeParticles0.velocity * (1 - edgeRatio) +
      edgeParticles1.velocity * edgeRatio;
  locus::Vector3 relativeV =
      particleColliding[0].velocity - virtualEdgeVelocity;
  real projectedVelocity = normal.dotProduct(relativeV);
  real edgeInverseMass =
      edgeParticles0.getInverseMass() + edgeParticles1.getInverseMass();
  real collidingParticleInverseMass = particleColliding->getInverseMass();
  real totalInverseMass = edgeInverseMass + collidingParticleInverseMass;
  real impulse = projectedVelocity; // / totalInverseMass;
  locus::Vector3 impulseVector = normal * impulse;

  /*DrawLineV({particleColliding->position.x, particleColliding->position.y},
            {particleColliding->position.x + normal.x * 1000,
             particleColliding->position.y + normal.y * 1000},
             WHITE);*/
  // resolve penetration
  move[0] = normal * penetration * 0.5f;
  particleColliding->position += move[0];

  real distributedEdgePenetration = penetration * 0.5f;
  move[1] = normal * (distributedEdgePenetration) * (1 - edgeRatio);
  edgeParticles0.position -= move[1];
  move[2] = normal * (distributedEdgePenetration)*edgeRatio;
  edgeParticles1.position -= move[2];
  //  add impulse to velocity vectors
  //  std::cout<<"V before "<<particleColliding->velocity.x<<"\n";
  if (projectedVelocity < -0.001f) {
    particleColliding->velocity -=
        (impulseVector)*particleColliding->getInverseMass();
    // std::cout<<"V after "<<particleColliding->velocity.x<<"\n";
    edgeParticles0.velocity +=
        (impulseVector * edgeInverseMass) * (1 - edgeRatio);
    edgeParticles1.velocity += (impulseVector * edgeInverseMass) * edgeRatio;
  }
}

struct AABB {
  MassAggregate *a;
  Particle *collidingParticle;

private:
  locus::Vector3 Vmax = locus::Vector3(std::numeric_limits<float>::min(),
                                       std::numeric_limits<float>::min(), 0.f);
  locus::Vector3 Vmin = locus::Vector3(std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max(), 0.f);

public:
  bool checkCollision() {
    bool isColliding = false;
    for (Particle &p : a->particles) {
      Vmax = locus::Vector3(std::max(p.position.x, Vmax.x),
                            std::max(p.position.y, Vmax.y), 0.f);
      Vmin = locus::Vector3(std::min(p.position.x, Vmin.x),
                            std::min(p.position.y, Vmin.y), 0.f);
    }
    const real boundingWidth = 0.1f;
    locus::Vector3 collidingParticleboundingBoxMax(
        collidingParticle->position.x + boundingWidth,
        collidingParticle->position.y + boundingWidth, 0);
    locus::Vector3 collidingParticleboundingBoxMin(
        collidingParticle->position.x - boundingWidth,
        collidingParticle->position.y - boundingWidth, 0);
    bool xWithin = collidingParticleboundingBoxMax.x <= Vmax.x &&
                   collidingParticleboundingBoxMin.x >= Vmin.x;
    bool yWithin = collidingParticleboundingBoxMax.y <= Vmax.y &&
                   collidingParticleboundingBoxMin.y >= Vmin.y;
    // DEBUG BOUNDING BOX
    if (false) {
      DrawLineEx({Vmin.x, Vmin.y}, {Vmax.x, Vmin.y}, 10, BLUE);
      DrawLineEx({Vmin.x, Vmax.y}, {Vmax.x, Vmax.y}, 10, BLUE);
      DrawLineEx({Vmin.x, Vmin.y}, {Vmin.x, Vmax.y}, 10, BLUE);
      DrawLineEx({Vmax.x, Vmin.y}, {Vmax.x, Vmax.y}, 10, BLUE);

      DrawLineEx({collidingParticleboundingBoxMin.x,
                  collidingParticleboundingBoxMin.y},
                 {collidingParticleboundingBoxMin.x,
                  collidingParticleboundingBoxMax.y},
                 10, GREEN);
      DrawLineEx({collidingParticleboundingBoxMax.x,
                  collidingParticleboundingBoxMin.y},
                 {collidingParticleboundingBoxMax.x,
                  collidingParticleboundingBoxMax.y},
                 10, GREEN);
      DrawLineEx({collidingParticleboundingBoxMin.x,
                  collidingParticleboundingBoxMin.y},
                 {collidingParticleboundingBoxMax.x,
                  collidingParticleboundingBoxMin.y},
                 10, GREEN);
      DrawLineEx({collidingParticleboundingBoxMin.x,
                  collidingParticleboundingBoxMax.y},
                 {collidingParticleboundingBoxMax.x,
                  collidingParticleboundingBoxMax.y},
                 10, GREEN);
    }
    isColliding = xWithin && yWithin;
    return isColliding;
  }
  real getShapeMaxX() const { return Vmax.x; }
  real getShapeMaxY() const { return Vmax.y; }
};

std::vector<ParticleEdgeContact> ParticleEdgeCollisionDetection::createContacts(
    Particle &collidingParticle, std::vector<MassAggregate *> shapes) {
  // DrawLineV({collidingParticle.position.x, collidingParticle.position.y},
  //         {rayEnd.x, rayEnd.y}, WHITE);

  std::vector<ParticleEdgeContact> generatedcontacts;
  for (MassAggregate *shape1 : shapes) {
    auto &shape_ = *shape1;
    real closestDistance = real_INFINITY;
    std::pair<unsigned, unsigned> closestEdgeRecord;
    locus::Vector3 collisionNormal;
    locus::Vector3 edgeStart;
    locus::Vector3 edgeEnd;
    //------------COLLISION CHECK------------------//
    // AABBB

    AABB aaBBcollisionChecker;
    aaBBcollisionChecker.a = shape1;
    aaBBcollisionChecker.collidingParticle = &collidingParticle;
    if (!aaBBcollisionChecker.checkCollision()) {
      continue;
    }

    // RAY CAST

    locus::Vector3 rayEndH = locus::Vector3(
        collidingParticle.position.x + aaBBcollisionChecker.getShapeMaxX(),
        collidingParticle.position.y, 0.f);
    locus::Vector3 rayEndV = locus::Vector3(
        collidingParticle.position.x + 1000.f,
        collidingParticle.position.y + aaBBcollisionChecker.getShapeMaxY(),
        0.f);
    locus::Vector3 allrays[] = {rayEndH, rayEndV};
    bool notInShape =false;
    for (size_t i = 0; i < 2; i++) {
      unsigned int numIntersections = 0;
      auto rayEnd = allrays[i];
      for (size_t i = 0; i < shape_.particles.size(); i++) {
        unsigned next = (i + 1) % shape_.particles.size();
        edgeStart = shape_.particles[i].position;
        edgeEnd = shape_.particles[next].position;
        if (LineInterSection(rayEnd, collidingParticle.position, edgeStart,
                             edgeEnd)) {
          numIntersections++;
         // std::cerr << i << " :intersection idx......" << i << ", " << next
           //         << "\n";
        }
      }
      //std::cerr << "numIntersects " << numIntersections << "\n";
      if (numIntersections % 2 == 0) {
        notInShape=true;
        break;
      }
    }
    if(notInShape){
        continue;
    }

    //----------END COLLISION CHECK ----------------------//
    for (size_t i = 0; i < shape_.particles.size(); i++) {
      unsigned next = (i + 1) % shape_.particles.size();
      edgeStart = shape_.particles[i].position;
      edgeEnd = shape_.particles[next].position;
      auto resultPair = closestEdge(collidingParticle, edgeStart, edgeEnd);
      real dist = resultPair.first;
      // we are probabably inline with a line
      if (dist > 1e6) {
        continue;
      }
      if (std::abs(dist) < std::abs(closestDistance)) {
        closestDistance = std::abs(dist);
        closestEdgeRecord = std::pair<unsigned, unsigned>(i, next);
        collisionNormal = (resultPair.second);
      }
    }
    ParticleEdgeContact contact;
    contact.penetration = closestDistance;
    contact.normal = collisionNormal;
    contact.particleColliding = &collidingParticle;
    contact.edgeParticleIndices[0] = closestEdgeRecord.first;
    contact.edgeParticleIndices[1] = closestEdgeRecord.second;
    contact.shape = shape1;
    generatedcontacts.push_back(contact);
  }
  return generatedcontacts;
}

bool ParticleEdgeCollisionDetection::LineInterSection(
    locus::Vector3 line1Start, locus::Vector3 line1End,
    locus::Vector3 line2Start, locus::Vector3 line2End) const {
  // line 1
  // y=mx+c
  // -mx+y-c
  //  ax+by+c
  // a = -m
  // a = y2-y1
  // b = x1-x2
  // c = a * x1 + b * y1
  real a1 = line1End.y - line1Start.y;
  real b1 = line1Start.x - line1End.x;
  real c1 = a1 * line1Start.x + b1 * line1Start.y;
  // line 2
  real a2 = line2End.y - line2Start.y;
  real b2 = line2Start.x - line2End.x;
  real c2 = a2 * line2Start.x + b2 * line2Start.y;
  real determinant = a1 * b2 - a2 * b1;
  if (determinant == 0.0f) {
    return false;
  }
  // check interception
  real x = (b2 * c1 - b1 * c2) / determinant;
  real y = (a1 * c2 - a2 * c1) / determinant;
  // are we within bounds
  if (x >= std::min(line1Start.x, line1End.x) &&
      x <= std::max(line1Start.x, line1End.x) &&
      y >= std::min(line1Start.y, line1End.y) &&
      y <= std::max(line1Start.y, line1End.y) &&
      x >= std::min(line2Start.x, line2End.x) &&
      x <= std::max(line2Start.x, line2End.x) &&
      y >= std::min(line2Start.y, line2End.y) &&
      y <= std::max(line2Start.y, line2End.y)) {

    return true;
  }

  return false;
}

std::pair<real, locus::Vector3>
ParticleEdgeCollisionDetection::closestEdge(Particle &collidingParticle,
                                            locus::Vector3 line2Start,
                                            locus::Vector3 line2End) const {
  locus::Vector3 edgev = line2End - line2Start;
  locus::Vector3 direction = edgev;
  direction.normalize();

  locus::Vector3 particleToEdge = collidingParticle.position - line2Start;
  real projection = particleToEdge.dotProduct(direction);
  real edgeLength = edgev.magnitude();
  real projectionRatio = projection / edgeLength;

  locus::Vector3 closePoint = line2Start;
  const real min = 0.f;
  if (projectionRatio < min) {
    return std::pair<real, locus::Vector3>(std::numeric_limits<float>::max(),
                                           locus::Vector3(1, 0, 0));
  }
  if (projectionRatio == min) {
    closePoint = line2Start;
  } else if (projectionRatio == 1.0f) {
    closePoint = line2End;
  } else if (projectionRatio > min && projectionRatio < 1.0f) {
    closePoint = line2Start;
    closePoint += direction * projection;
  }

  real distanceToClosestPoint =
      (collidingParticle.position - closePoint).magnitude();
  // DrawCircle(closePoint.x, closePoint.y, 30, BLACK);
  locus::Vector3 normalizedV = (closePoint - collidingParticle.position);
  normalizedV.normalize();
  return std::pair<real, locus::Vector3>(distanceToClosestPoint, normalizedV);
}
bool ParticleEdgeCollisionDetection::checkEdgeEdgeCollision(
    locus::Vector3 &edgeStart, locus::Vector3 &edgeEnd,
    locus::Vector3 &collidingParticleposition, locus::Vector3 &particlePrevPos,
    locus::Vector3 &edgecollisionNormal, real &edgeCollisionPenetration) const {
  // Pain
  auto line1Start = edgeStart;
  auto line1End = edgeEnd;
  auto line2Start = collidingParticleposition;
  auto line2End = particlePrevPos;

  bool line1ContainedInLine2 =
      std::min(line1Start.x, line1End.x) >=
          std::min(line2Start.x, line2End.x) &&
      std::max(line1Start.x, line1End.x) <=
          std::max(line2Start.x, line2End.x) &&
      std::min(line1Start.y, line1End.y) >=
          std::min(line2Start.y, line2End.y) &&
      std::max(line1Start.y, line1End.y) <= std::max(line2Start.y, line2End.y);

  // Check if line2 is contained within line1
  bool line2ContainedInLine1 =
      std::min(line2Start.x, line2End.x) >=
          std::min(line1Start.x, line1End.x) &&
      std::max(line2Start.x, line2End.x) <=
          std::max(line1Start.x, line1End.x) &&
      std::min(line2Start.y, line2End.y) >=
          std::min(line1Start.y, line1End.y) &&
      std::max(line2Start.y, line2End.y) <= std::max(line1Start.y, line1End.y);
  std::cout << line1ContainedInLine2 << " ," << line2ContainedInLine1 << "\n";
  if (line1ContainedInLine2 || line2ContainedInLine1) {
    locus::Vector3 a = collidingParticleposition - edgeStart;
    locus::Vector3 b = collidingParticleposition - edgeEnd;
    locus::Vector3 c = a.squareMagnitude() < b.squareMagnitude() ? a : b;
    edgecollisionNormal = c;
    edgeCollisionPenetration = c.magnitude();
    edgecollisionNormal.normalize();
    DrawLineV({collidingParticleposition.x, collidingParticleposition.y},
              {collidingParticleposition.x + 240 * edgecollisionNormal.x,
               collidingParticleposition.y + 240 * edgecollisionNormal.y},
              WHITE);
    DrawCircleV({collidingParticleposition.x, collidingParticleposition.y}, 20,
                WHITE);
    return true;
  }
  return false;
}
ParticleCollisionReSolver::ParticleCollisionReSolver(size_t numParticles) {
  ParticleCollisionReSolver::iterations = numParticles;
  ParticleCollisionReSolver::iterationsUsed = 0;
}
void ParticleCollisionReSolver::solveCollision(
    ParticleContact *particleContacts, size_t size, real dt) {

  iterationsUsed = 0;
  if (particleContacts == nullptr)
    return;
  if (!size)
    return;
  while (iterationsUsed < iterations) {
    locus::real maxSeperatingV = real_INFINITY;
    int maxIndex = size;
    for (size_t i = 0; i < size; i++) {
      real sepV = particleContacts[i].calculateSeparatingV();
      if (sepV < maxSeperatingV) {
        maxSeperatingV = sepV;
        maxIndex = i;
      }
    }
    if (maxIndex == size)
      break;
    particleContacts[maxIndex].resolve(dt);
    locus::Vector3 *movement = particleContacts[maxIndex].move;

    for (size_t i = 0; i < size; i++) {
      if (particleContacts[i].p[0] == particleContacts[maxIndex].p[0]) {
        particleContacts[i].penetration -=
            (movement[0]).dotProduct(particleContacts[i].contactNormal);
      }

      if (particleContacts[i].p[0] == particleContacts[maxIndex].p[1]) {
        particleContacts[i].penetration -=
            (movement[1]).dotProduct(particleContacts[i].contactNormal);
      }

      if (particleContacts[i].p[1]) {
        if (particleContacts[i].p[1] == particleContacts[maxIndex].p[0]) {
          particleContacts[i].penetration +=
              (movement[0]).dotProduct(particleContacts[i].contactNormal);
        }
        if (particleContacts[i].p[1] == particleContacts[maxIndex].p[0]) {
          particleContacts[i].penetration +=
              (movement[1]).dotProduct(particleContacts[i].contactNormal);
        }
      }
    }

    iterationsUsed++;
  }
}

ParticleEdgeCollisionReSolver::ParticleEdgeCollisionReSolver(
    size_t numiterations) {
  iterations = numiterations;
  iterationsUsed = 0;
}
void ParticleEdgeCollisionReSolver::solveCollision(
    std::vector<ParticleEdgeContact> &edgecontacts, real dt) {
  if (edgecontacts.size() == 0)
    return;
  while (iterationsUsed < iterations) {
    real maxPenetration = std::numeric_limits<float>::min();
    real maxIndex;
    for (size_t i = 0; i < edgecontacts.size(); i++) {
      if (std::abs(edgecontacts[i].penetration) > maxPenetration) {
        maxIndex = i;
      }
    }
    edgecontacts[maxIndex].resolve(dt);

    for (size_t i = 0; i < edgecontacts.size(); i++) {
      if (edgecontacts[i].shape != edgecontacts[maxIndex].shape) {
        continue;
      }
      if (edgecontacts[i].particleColliding ==
          edgecontacts[maxIndex].particleColliding) {
        edgecontacts[i].penetration -=
            edgecontacts[maxIndex].move[0].dotProduct(edgecontacts[i].normal);
      }
      if (edgecontacts[i].edgeParticleIndices[0] ==
          edgecontacts[maxIndex].edgeParticleIndices[0]) {
        edgecontacts[i].penetration -=
            edgecontacts[maxIndex].move[1].dotProduct(edgecontacts[i].normal);
      }
      if (edgecontacts[i].edgeParticleIndices[1] ==
          edgecontacts[maxIndex].edgeParticleIndices[1]) {
        edgecontacts[i].penetration -=
            edgecontacts[maxIndex].move[2].dotProduct(edgecontacts[i].normal);
      }
    }
    iterationsUsed++;
  }
  iterationsUsed = 0;
}
