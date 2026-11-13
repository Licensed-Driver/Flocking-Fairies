#[compute]
#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_scalar_block_layout : require
#include "boid.head"

layout(local_size_x=128, local_size_y=1, local_size_z=1) in;

void main() {
    uint id = gl_GlobalInvocationID.x;
    if (id >= boids.length()) return;

    Boid us = boids[id];

    // Bound just in case we have tiny impresisions or something
    boids[id].Transform.Partition = GetPartition(us.Transform.Position);
    boids[id].Transform.PartRelIndex = atomicAdd(partCounts[us.SchoolOffset + us.Transform.Partition], 1);
}