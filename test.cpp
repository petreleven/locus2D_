#include <algorithm>

struct AABB {
    locus::Vector3 min;
    locus::Vector3 max;
};

AABB computeAABB(const MassAggregate& shape) {
    AABB aabb;
    aabb.min = aabb.max = shape.particles[0].position;
    for (const auto& particle : shape.particles) {
        aabb.min.x = std::min(aabb.min.x, particle.position.x);
        aabb.min.y = std::min(aabb.min.y, particle.position.y);
        aabb.max.x = std::max(aabb.max.x, particle.position.x);
        aabb.max.y = std::max(aabb.max.y, particle.position.y);
    }
    return aabb;
}

bool checkAABBOverlap(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y);
}

bool checkEdgeEdgeCollision(const locus::Vector3& p1, const locus::Vector3& p2,
                            const locus::Vector3& p3, const locus::Vector3& p4,
                            locus::Vector3& collisionPoint) {
    locus::Vector3 s1 = p2 - p1;
    locus::Vector3 s2 = p4 - p3;

    float s, t;
    s = (-s1.y * (p1.x - p3.x) + s1.x * (p1.y - p3.y)) / (-s2.x * s1.y + s1.x * s2.y);
    t = ( s2.x * (p1.y - p3.y) - s2.y * (p1.x - p3.x)) / (-s2.x * s1.y + s1.x * s2.y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        collisionPoint = p1 + t * s1;
        return true;
    }

    return false;
}

std::vector<ParticleEdgeContact> ParticleEdgeCollisionDetection::createContacts(
    Particle &collidingParticle, std::vector<MassAggregate *> shapes) {
    std::vector<ParticleEdgeContact> generatedcontacts;

    for (MassAggregate *shape1 : shapes) {
        auto shape = *shape1;

        // Broad-phase collision detection using AABB
        AABB particleAABB = {{collidingParticle.position.x - 1, collidingParticle.position.y - 1},
                             {collidingParticle.position.x + 1, collidingParticle.position.y + 1}};
        AABB shapeAABB = computeAABB(shape);

        if (!checkAABBOverlap(particleAABB, shapeAABB)) {
            continue;
        }

        // Detailed collision detection
        unsigned numIntersections = 0;
        real closestDistance = real_INFINITY;
        std::pair<unsigned, unsigned> closestEdgeRecord;
        locus::Vector3 collisionNormal;

        for (size_t i = 0; i < shape.particles.size(); i++) {
            locus::Vector3 edgeStart = shape.particles[i].position;
            locus::Vector3 edgeEnd = shape.particles[(i + 1) % shape.particles.size()].position;

            // Check for edge-particle collision
            auto resultPair = closestEdge(collidingParticle, edgeStart, edgeEnd);
            real dist = resultPair.first;

            if (std::abs(dist) < closestDistance) {
                closestDistance = std::abs(dist);
                closestEdgeRecord = std::pair<unsigned, unsigned>(i, (i + 1) % shape.particles.size());
                collisionNormal = resultPair.second;
            }

            // Check for edge-edge collision with particle's path
            locus::Vector3 particlePrevPos = collidingParticle.position - collidingParticle.velocity;
            locus::Vector3 collisionPoint;
            if (checkEdgeEdgeCollision(particlePrevPos, collidingParticle.position, edgeStart, edgeEnd, collisionPoint)) {
                numIntersections++;
                real dist = (collisionPoint - collidingParticle.position).magnitude();
                if (dist < closestDistance) {
                    closestDistance = dist;
                    closestEdgeRecord = std::pair<unsigned, unsigned>(i, (i + 1) % shape.particles.size());
                    collisionNormal = (edgeEnd - edgeStart).cross(locus::Vector3(0, 0, 1)).normalize();
                }
            }
        }

        if (numIntersections > 0 || closestDistance < collidingParticle.radius) {
            ParticleEdgeContact contact;
            contact.penetration = closestDistance;
            contact.normal = collisionNormal;
            contact.particleColliding = &collidingParticle;
            contact.edgeParticleIndices[0] = closestEdgeRecord.first;
            contact.edgeParticleIndices[1] = closestEdgeRecord.second;
            contact.shape = shape1;
            generatedcontacts.push_back(contact);
        }
    }

    return generatedcontacts;
}
