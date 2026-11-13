#[compute]
#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_scalar_block_layout : require

#include "boid.head"

layout(local_size_x=128, local_size_y=1, local_size_z=1) in;

// Each thread calculate partition and incrememnt the partition counts, last thread make the counts into partition indices
void main() {
    uint id = gl_GlobalInvocationID.x;
    if (id >= boids.length()) return;
    // count_parts and sum_parts should've already run, so now we just gotta sort ourselves
    // We reset PartRelIndex in count_parts so we just add where our partition starts in sortedBoids
    Boid us = boids[id];
    us.Transform.PartRelIndex += us.SchoolOffset + partCounts[us.Transform.Partition];
    sortedBoids[us.Transform.PartRelIndex] = us;
    boids[id] = us;
}