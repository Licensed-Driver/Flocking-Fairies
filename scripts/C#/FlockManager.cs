using Godot;
using System;
using System.Linq;
using System.Net;
using System.Reflection.Metadata;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

[StructLayout(LayoutKind.Sequential, Pack = 4)]
public struct Transform3D
{
	public Vector3 Position;
	public uint PartRelIndex;
    public Vector3 Velocity;
    public uint Partition;
    public Vector3 Acceleration;
    public float Radius;
    public Vector4 Rotation;
    public Vector3 GlobalPosition;
	private float _padding1;
    public Vector3 DesiredHeading;
	private float _padding2;
}

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct SphereCollider3D {
    public float Radius;
};

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct RuleRadii {
    public float SeparationRad;
    public float AlignmentRad;
    public float CohesionRad;
};

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct RuleCoefficients {
    public float SeparationCoef;
    public float AlignmentCoef;
    public float CohesionCoef;
};

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct RuleSettings {
    public RuleRadii Radii;
    public RuleCoefficients Coefficients;
};

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct GpuBoid {
    public Transform3D Transform;
    public SphereCollider3D Collider;
    public RuleSettings Settings;
    public float cosFov;
	public uint School;
    public uint Index;
};

[StructLayout(LayoutKind.Sequential, Pack = 4)]
struct GpuPlanet
{
	public Vector3 Centre;
    public float Radius;   // The radius is from center to the sky limit
    public float Gravity;
    public uint NumPartitions;
    public uint PartitionsPerAxis;
    public float PartitionSize;
	// For debug testing, removed when we use SDF
	public float GroundRadius;
}

public partial class FlockManager : Node3D
{
	[Export] public int[] SchoolCounts = {100, 100, 100};
	[Export] public float MaximumAltitude = 50.0f;
	[Export] public float SearchRadius = 5.0f;
	[Export] public float Speed = 0.001f;
	[Export] public float Accel = 5.0f;
	[Export] public float Decel = 15.0f;
	[Export] public float FieldOfView = 90.0f;
	[Export] public World Planet;
	[Export] public Vector2 SpawnHeight = new Vector2(5.0f, 15.0f);
	[Export] public float NeighbourStrength = 0.5f;
	[Export] public float TurnRate = 100.0f;
	[Export] public float SeparationRadius = 5.0f;
	[Export] public float AlignmentRadius = 15.0f;
	[Export] public float CohesionRadius = 15.0f;
	[Export] public float SeparationCoefficient = 1.0f;
	[Export] public float AlignmentCoefficient = 1.0f;
	[Export] public float CohesionCoefficient = 1.0f;
	public int Schools => SchoolCounts.Length;

	// Groups of boids so we have distinct flocks or schools
	private Boid[][] boids;
	private Boid[] allBoids;   // View into 2d for flattened indexing
	private int[] schoolOf;    // which school each flat boid belongs to
	private MultiMesh multiMesh;

	// GPU version
	private GpuPlanet gpuPlanet;
	private GpuBoid[][] gpuBoids;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		// Get multimesh and tranform information setup
		multiMesh = GetNode<MultiMeshInstance3D>("MultiMeshInstance3D").Multimesh;
		multiMesh.TransformFormat = MultiMesh.TransformFormatEnum.Transform3D;

		float partitionSize = 2f * Mathf.Max(SeparationRadius, Mathf.Max(AlignmentRadius, CohesionRadius));
		uint partsPerAxis = (uint) Mathf.Ceil(2f*Planet.Radius/partitionSize);
		uint numPartitions = (uint) Mathf.Pow(partsPerAxis, 3);
		gpuPlanet = new GpuPlanet
		{
			Centre = Planet.GlobalPosition,
			Radius = Planet.Scale.X/2f,
			Gravity = Planet.Gravity,
			// Largest search radius decides partition size
			NumPartitions = numPartitions,
			PartitionsPerAxis = partsPerAxis,
			PartitionSize = partitionSize,
			GroundRadius = Planet.Scale.X/2f
		};

		gpuBoids = new GpuBoid[Schools][];
		for (int i = 0; i < Schools; i++) gpuBoids[i] = new GpuBoid[SchoolCounts[i]];
		// Generate a bunch of random starting positions and rotations around the planet
		RandomNumberGenerator gpuRng = new RandomNumberGenerator();
		for (uint i = 0; i < SchoolCounts.Length; i++)
		{
			for (uint j = 0; j < SchoolCounts[i]; j++)
			{
				Vector3 dir = new Vector3(gpuRng.RandfRange(-3.1f, 3.1f), gpuRng.RandfRange(-3.1f, 3.1f), gpuRng.RandfRange(-3.1f, 3.1f)).Normalized();	// Random rotation to pick locations for Boids
				Vector3 position = Planet.GlobalPosition + dir * (Planet.Scale.X*0.5f + gpuRng.RandfRange(SpawnHeight[0], SpawnHeight[1]));
				gpuBoids[i][j] = new GpuBoid
				{
					Transform = new Transform3D
					{
						Position = position,
						PartRelIndex = 0,	// Set by compute shader before sorting
						Velocity = Vector3.Forward * Speed,
						Partition = 0,	// Also set by shader
						Acceleration = Vector3.Zero,
						Radius = 125f,	// From planet centre
						Rotation = Vector4.One,
						GlobalPosition = position+Planet.GlobalPosition,
						DesiredHeading = Vector3.Forward
					},
					Collider = new SphereCollider3D
					{
						Radius = 0.5f
					},
					Settings = new RuleSettings
					{
						Radii = new RuleRadii
						{
							SeparationRad = SeparationRadius,
							AlignmentRad = AlignmentRadius,
							CohesionRad = CohesionRadius
						},
						Coefficients = new RuleCoefficients
						{
							SeparationCoef = SeparationCoefficient,
							AlignmentCoef = AlignmentCoefficient,
							CohesionCoef = CohesionCoefficient
						}
					},
					cosFov = Mathf.Cos(FieldOfView/2.0f),
					School = i,
					Index = j
				};
			}
		}
		allBoids = boids.SelectMany(s => s).ToArray();
		schoolOf = new int[allBoids.Length];
		for (int s = 0, k = 0; s < Schools; s++)
			for (int j = 0; j < SchoolCounts[s]; j++) schoolOf[k++] = s;
		multiMesh.InstanceCount = allBoids.Length;	// all boids exist now


		boids = new Boid[Schools][];
		for (int i = 0; i < Schools; i++) boids[i] = new Boid[SchoolCounts[i]];

		// Generate a bunch of random starting positions and rotations around the planet
		RandomNumberGenerator rng = new RandomNumberGenerator();
		for (int i = 0; i < SchoolCounts.Length; i++)
		{
			for (int j = 0; j < SchoolCounts[i]; j++)
			{
				Vector3 dir = new Vector3(rng.RandfRange(-3.1f, 3.1f), rng.RandfRange(-3.1f, 3.1f), rng.RandfRange(-3.1f, 3.1f)).Normalized();	// Random rotation to pick locations for Boids
				Vector3 position = Planet.GlobalPosition + dir * (Planet.Scale.X*0.5f + rng.RandfRange(SpawnHeight[0], SpawnHeight[1]));
				boids[i][j] = new Boid(position, Planet, Vector3.Forward, 20, turnRate:TurnRate)
				{
					Velocity = Vector3.Up * Speed
				};
				boids[i][j].SetRuleRadii(SeparationRadius, AlignmentRadius, CohesionRadius);
				boids[i][j].SetRuleCoefficients(SeparationCoefficient, AlignmentCoefficient, CohesionCoefficient);
				boids[i][j].FieldOfView = FieldOfView;
			}
		}
		allBoids = boids.SelectMany(s => s).ToArray();
		schoolOf = new int[allBoids.Length];
		for (int s = 0, k = 0; s < Schools; s++)
			for (int j = 0; j < SchoolCounts[s]; j++) schoolOf[k++] = s;
		multiMesh.InstanceCount = allBoids.Length;	// all boids exist now


	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	public override void _PhysicsProcess(double delta)
	{
		float dt = (float)delta;
		var world = GetWorld3D();

		// Perform any tasks that would require reaching over Godot's multithread boundary on the main thread
		var query = PhysicsRayQueryParameters3D.Create(Vector3.Zero, Vector3.Zero);	// For here isn't parallel so each gets to use the query indipendently b
		query.CollisionMask = 0b0000000000000010;   // Layer 2 for planets
		foreach (Boid boid in allBoids) boid.MainThreadTasks(world, query); 

		Parallel.For(0, allBoids.Length, i => allBoids[i].ProcessBoids(boids[schoolOf[i]]));

		Parallel.For(0, allBoids.Length, i =>
		{
			allBoids[i].ApplyHeading(dt);
			allBoids[i].Simulate(dt);
		});

		for (int i = 0; i < allBoids.Length; i++)
			multiMesh.SetInstanceTransform(i, new Transform3D(allBoids[i].RotationBasis, allBoids[i].GlobalPosition));
	}
}
