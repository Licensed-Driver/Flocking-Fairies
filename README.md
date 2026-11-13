# Flocking Fairies

I realized that I stopped making things that I thought were fun to make, instead opting for things that I thought would help my career once I finish. Obviously that hasn't led anywhere but burnout town, so I wanted to start something that was actually fun to make. \
Boids sounded fun, and I thought making them little fairies to catch would be fun as a little game. I love games like Outer Wilds with the planets and gravity, so I wanted to do that too. So I started making Flocking Fairies in Godot 4 (C#). I made the initial logic on the CPU but that didn't run as well as I wanted, so I have been porting the logic to the GPU.

> **Status:** Work in progress. The GPU compute pipeline is getting there but still being built.

## Current Intended Features

- **GPU boids.** Separation, alignment, and cohesion run in GLSL compute shaders.
- **Spatial partitioning.** Boids are sorted into a grid of partitions, so each boid only checks nearby neighbours.
- **Multiple schools.** Boids a split into schools, only comparing against the same school to guarantee distinct flocks or "Schools" and increase performance.
- **Spherical world.** Gravity and orientation are calculated relative to the planet's centre.
- **First-person player** Regular mouse look and WASD movement, gamepad support. Player has no gravity, space to jump and shift to jump back down to the planet, so that you can fly around and look at the boids.

## Requirements

- [Godot 4.7](https://godotengine.org/download) **.NET edition** (needed for C#)
- [.NET 8 SDK](https://dotnet.microsoft.com/download)
- A GPU with Vulkan support (the project uses the Forward+ renderer)

## Trying It Out

1. Clone the repo:
   ```sh
   git clone git@github.com:Licensed-Driver/Flocking-Fairies.git
   cd flocking-fairies
   ```
2. Open project.godot in Godot
3. Run the main scene (should be `node_3d.tscn`)

## Controls

| Action        | Keyboard / Mouse | Gamepad     |
| ------------- | ---------------- | ----------- |
| Move          | W A S D          | Left stick  |
| Jump          | Space            | —           |
| Look around   | Mouse            | —           |
| Jump Down     | Shift            | -           |

## Current File Structure and Functinos

```
scripts/
├── C#/
│   ├── FlockManager.cs     # Sets up GPU buffers and dispatches the shaders
│   ├── Boid.cs             # Will hold relevant boid data on CPU side
│   ├── World.cs            # Planet: radius, gravity, planet-relative bases
│   ├── WorldStaticBody.cs  # General world collision physics
│   ├── Player.cs           # First person controller
│   └── Utils.cs
└── GLSL/
    ├── boid.head           # Shared structs, buffers, and helpers (included by every shader)
    ├── count_parts.glsl    # Pass 1: assign each boid a partition and count boids per partition
    ├── sum_parts.glsl      # Pass 2: prefix sum, turning counts into start offsets
    ├── partition.glsl      # Pass 3: write boids into a buffer sorted by partition
    └── boid_rules.glsl     # Pass 4: apply the flocking rules using neighbouring partitions
skybox/                     # Skybox textures
node_3d.tscn                # Main scene
```

## Current Process

This is the current architecture, but was done without schools as a simple GPU implementation first. Adding schools made the Count sort more complex on memory, so I'm currently working on the best time to memory trade off.

Each frame, `FlockManager` runs the compute passes in this order:

1. **Count.** Each boid works out which partition it is in and atomically increments that partition's count.
2. **Sum.** A parallel prefix sum turns the per partition counts into starting indices.
3. **Sort.** Each boid copies itself into `sortedBoids` at its partition's start index plus its own offset within the partition.
4. **Rules.** Each boid checks the boids in the (up to 8) neighbouring partitions and applies separation, alignment, and cohesion, limited by its field of view.
