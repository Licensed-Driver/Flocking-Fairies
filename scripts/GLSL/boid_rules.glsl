#[compute]
#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_scalar_block_layout : require

#include "boid.head"

void processRules(uint id);
void getNeighbourParts(vec3 position, uint partitionId, out uint[8] neighbourKeys);
vec3 quatRotate(vec4 quat, vec3 vec);
bool canSeeBoid(float cosFov, vec3 globalHeading, vec3 usPos, vec3 otherPos);
float angleTo(vec3 from, vec3 to);
vec3 slide(vec3 vec, vec3 normal);

// Entry point, one per thread
void main() {
    uint globalId = gl_GlobalInvocationID.x;
    if (globalId >= boids.length()) return;


}

void getNeighbourParts(vec3 position, uint partitionId, out uint[8] neighbourKeys) {
    // Get all partitions that we have a chance of interacting with
    // Guaranteed to be 1-4 since partition size is boid radius
    // Depending on position in partition, if we are to the right, then we can interact with the right, to the left then the left and so on
    vec3 partOffset = position - GetPartitionCentre(partitionId); // Get offset from centre of partition
    // Anything at 0 or above is 1 and anything negative is -1
    partOffset = ivec3(step(0.0, partOffset)) * 2 - 1;
    for (int i = 0; i < 8; i++) {
        // Using bits flipping as permutations of the offset directions
        ivec3 pick = ivec3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        neighbourKeys[i] = GetPartition(position + pick * partOffset);
    }
}

// From local to global
vec3 quatRotate(vec4 quat, vec3 vec) {
    vec3 temp = 2.0 * cross(quat.xyz, vec);
    return vec + quat.w * temp + cross(quat.xyz, temp);
}

// From global to local
vec3 quatConjRotate(vec4 quat, vec3 vec) {
    return quatRotate(vec4(-quat.xyz, quat.w), vec);
}

// Take in global heading since it doesn't change per neighbour check
bool canSeeBoid(float cosFov, vec3 globalHeading, vec3 usPos, vec3 otherPos) {
    // Normalize the diff and heading is already normal so we don't need to divide by lengths ourselves
    vec3 diff = normalize(otherPos - usPos);
    float cosine = dot(globalHeading, diff);
    return cosine <= cosFov;
}

float angleTo(vec3 from, vec3 to) {
    return acos(dot(normalize(from), normalize(to)));
}

vec3 slide(vec3 vec, vec3 normal) {
    return vec - dot(vec, normal) * normal;
}

void processRules(uint thread) {
    // Separation, alignment, and cohesion acceleration vectors
    vec3 sVec = vec3(0, 0, 0);
    vec3 aVec = vec3(0, 0, 0);
    vec3 cVec = vec3(0, 0, 0);
    int aCount = 0;
    int cCount = 0;
    uint neighbours[8];

    uint school = 0;
    // For each school that we are too high for we skip and set our school when we finaly fit
    for (uint i = 0; i < schoolCounts.length(); i++) {
        uint count = schoolCounts[i];
        if (thread >= count) thread -= count;
        else
        {
            school = i;
            break;
        }
    }

    // Since we subtracted, thread is now the index within the school
    Boid us = sortedBoids[thread];

    getNeighbourParts(us.Transform.Position, us.Transform.Partition, neighbours);

    vec3 localFwd = vec3(0, 0, -1);
    vec3 globalHeading = quatRotate(us.Transform.Rotation, localFwd);
    vec3 usUp = normalize(us.Transform.Position);
    // Altitude is gonna be gotten from SDF of noise map used for terrain generation
    // Since we're just using a sphere rn we can do simple equation
    float altitude = length(us.Transform.Position) - planet.GroundRadius;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < partCounts[us.SchoolOffset + neighbours[i]]; j++) {
            Boid boid = sortedBoids[us.SchoolOffset + partIdx[us.SchoolOffset + neighbours[i]] + j];
            // Skip ourselves
            if (boid.Index == us.Index) continue;
            // Only process if we can see the other boid
            if (!canSeeBoid(us.cosFov, globalHeading, us.Transform.Position, boid.Transform.Position)) continue;
            vec3 themUp = normalize(boid.Transform.Position);
            float distanceTo;
            // Great circle direction
            vec3 sphericalDirection;
            // Using our own radius since we account for altitude later
            float arclength = angleTo(us.Transform.Position, boid.Transform.Position) * us.Transform.Radius;
            if (arclength < 1e-8)	// Other boid is directly above or below on this side of planet
            {
                distanceTo = length(boid.Transform.Position - us.Transform.Position);
                sphericalDirection = normalize(boid.Transform.Position - us.Transform.Position);
            } else	// Other boid is other side of planet
            {
                distanceTo = sqrt(arclength*arclength + (boid.Transform.Radius - us.Transform.Radius)*(boid.Transform.Radius - us.Transform.Radius));
                sphericalDirection = normalize(normalize(slide(boid.Transform.Position, usUp)) * arclength + usUp * (boid.Transform.Radius - us.Transform.Radius));
            }

            // All guaranteed normal or normalized
            if (distanceTo <= us.Settings.Radii.SeparationRad)
            {
                sVec -= sphericalDirection / (distanceTo*distanceTo);
            }
            if (distanceTo <= us.Settings.Radii.AlignmentRad)
            {
                aCount++;
                vec3 themFwd = quatRotate(boid.Transform.Rotation, localFwd);
                // Their pitch relative to their own surface
                float sinPitch = dot(themFwd, themUp);
                float cosPitch = length(slide(themFwd, themUp));
                // Get how where they are going forward in our tangent plane
                vec3 forwardRelToUs = slide(themFwd, usUp);
                if (length(forwardRelToUs) >= 1e-3) forwardRelToUs = normalize(forwardRelToUs) * cosPitch;
                // Get how much they are going up in their plane
                forwardRelToUs += usUp * sinPitch;
                aVec += forwardRelToUs;
            }
            if (distanceTo <= us.Settings.Radii.CohesionRad)
            {
                cCount++;
                cVec += sphericalDirection * distanceTo;
            }
        }
    }
    vec3 steer = quatConjRotate(us.Transform.Rotation, sVec) * us.Settings.Coefficients.SeparationCoef;
    if (aCount > 0) steer += quatConjRotate(us.Transform.Rotation, (aVec/aCount)) * us.Settings.Coefficients.AlignmentCoef;
    if (cCount > 0) steer += normalize(quatConjRotate(us.Transform.Rotation, (cVec/cCount))) * us.Settings.Coefficients.CohesionCoef;
    // add em up
    vec3 desiredHeading = length(steer) > 1e-3 ? steer : quatConjRotate(us.Transform.Rotation, usUp);
    if (altitude < 0)   // Raycast missed, so we just negate any climb or descent
    {
        desiredHeading = slide(desiredHeading, vec3(0, 1, 0));
    }
    vec3 normDesHead = normalize(desiredHeading);
    // Planet space so Y is local up
    // Velocity.Y is forward rn, also technically boids could translate up/down and side-to-side out of altitude, but they never do those so this works until they do
    if (normDesHead.y * us.Transform.Velocity.y + altitude > planet.Radius)  // If we go above maxalt we correct by forcing local down
    {
        desiredHeading.y = 0;
        desiredHeading = normDesHead;
        desiredHeading.y = -1;
    }
    if (normDesHead.y * us.Transform.Velocity.y + altitude < 0)
    {
        desiredHeading.y = 0;
        desiredHeading = normDesHead;
        desiredHeading.y = 1;
    }
    us.Transform.DesiredHeading = desiredHeading;

    boids[thread] = us; // In case we wanna add anything other than changing heading later
}
