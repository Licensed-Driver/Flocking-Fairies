#[compute]
#version 450
#extension GL_EXT_scalar_block_layout : require

#include "boid.head"

layout(local_size_x=1024, local_size_y=1, local_size_z=1) in;
shared uint totals[1024];

void main() {
    uint id = gl_GlobalInvocationID.x;

    
}

void SumSchoolParts(uint school, uint id) {
    // If we have more threads than partitions chunk size rounds up to 1
    uint chunkSize = uint(ceil(partCounts.length()/1024));
    uint sum = 0;

    // For each partition in your chunk less than N partitions, add each value to the count
    for (uint i = id*chunkSize; i < (id+1)*(chunkSize); i++) {
        if (i >= partCounts.length()) break;
        sum += partCounts[i];
    }

    // Since sum starts at 0, any threads past N partitions just set total to 0
    totals[id] = sum;

    barrier();

    // Since we always have 1024 values to sum, we know exactly how many loops we need
    for (int i = 1; i <= 10; i++) {
        // If we are a power of 2^i then we are on the i stride
        if (mod((id+1), pow(2, i)) == 0) {
            // Always a whole num, and guaranteed positive cuz of the if
            uint left = id-uint(pow(2, i-1));
            // Sum the right of each pair with the left into the right
            totals[id] += totals[left];
        }
        barrier();
    }

    if (id==0) {
        // Zero the last real element for the exclusive scan
        totals[max(partCounts.length()-1, 1023)] = 0;
    }
    barrier();

    for (int i = 1; i <= 10; i++) {
        if (mod((id+1), pow(2, i)) == 0) {
            // Use i-1 since if we are on the 2s we wanna walk 1 over, if we are on the 8s we wanna walk 4 over, etc.
            uint left = id-uint(pow(2, i-1));
            uint t = totals[left];
            totals[left] = totals[id];
            totals[id] += t;
        }
        barrier();
    }

    uint chunkSum = totals[id];
    for (uint i = id*chunkSize; i < (id+1)*chunkSize; i++) {
        if (i >= partCounts.length()) break;
        partIdx[i] = chunkSum;
        chunkSum+=partCounts[i];
    }
    // Should all be summed now! Hopefully
}