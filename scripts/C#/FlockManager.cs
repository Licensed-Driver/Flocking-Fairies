using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Reflection.Metadata;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics.Arm;
using System.Threading.Tasks;

public struct ShaderInfo
{
	public Rid Shader;
	public Rid Pipeline;

	public ShaderInfo(RenderingDevice rd, string path)
	{
		var spirv = GD.Load<RDShaderFile>(path).GetSpirV();
		Shader = rd.ShaderCreateFromSpirV(spirv);
		Pipeline =  rd.ComputePipelineCreate(Shader);
	}
}

public struct RunnableShaderInfo
{
	public Rid Shader;
	public Rid Pipeline;
	public uint Threads;

	public RunnableShaderInfo(RenderingDevice rd, string path, uint threads)
	{
		var spirv = GD.Load<RDShaderFile>(path).GetSpirV();
		Shader = rd.ShaderCreateFromSpirV(spirv);
		Pipeline =  rd.ComputePipelineCreate(Shader);
		Threads = threads;
	}
}

public struct Shaders
{
	// For things that will be run
	private Dictionary<string, RunnableShaderInfo> runnable = new Dictionary<string, RunnableShaderInfo>();
	// For things that are purely for tracking so we can free later
	private Dictionary<string, ShaderInfo> value = new Dictionary<string, ShaderInfo>();
	// Maintains insertion order so we run in order
	private List<string> execOrder = new List<string>();
	// A shared layout set to bind to each pipeline
	private Rid SharedSet;
	RenderingDevice Rd;

	public Shaders(RenderingDevice rd, Tuple<string, uint>[] runnableFiles, string[] valueFiles, Rid sharedSet)
	{
		foreach ((string filename, uint threadCount) in runnableFiles)
		{
			runnable.Add(filename, new RunnableShaderInfo(rd, $"res://scripts/GLSL/{filename}", threadCount));
			execOrder.Add(filename);	// Makes sure order is preserved since .NET doesn't guarantee for regular Dictionary and typed ordered dicts only in .NET 9
		}

		foreach (string filename in valueFiles)
		{
			value.Add(filename, new ShaderInfo(rd, $"res://scripts/GLSL/{filename}"));
		}

		SharedSet = sharedSet;
		Rd = rd;
	}

	public void Dispatch()
	{
		long list = Rd.ComputeListBegin();

		foreach (string name in execOrder)
		{
			Rd.ComputeListBindComputePipeline(list, runnable[name].Pipeline);
			Rd.ComputeListBindUniformSet(list, SharedSet, 0);
			Rd.ComputeListDispatch(list, Groups(runnable[name].Threads, 64u), 1, 1);
			Rd.ComputeListAddBarrier(list);
		}

		Rd.ComputeListEnd();
	}

	public void FreeAll()
	{
		foreach (RunnableShaderInfo shaderInfo in runnable.Values)
		{
			Rd.FreeRid(shaderInfo.Shader);
		}
		foreach (ShaderInfo shaderInfo in value.Values)
		{
			Rd.FreeRid(shaderInfo.Shader);
		}
	}

	static uint Groups(uint n, uint localSize) => (uint)((n + localSize - 1) / localSize);
}

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
	private GpuBoid[] Boids;   // View into 2d for flattened indexing
	private int[] schoolOf;    // which school each flat boid belongs to
	private MultiMesh multiMesh;

	// GPU Specific Objects
	private RenderingDevice rd = RenderingServer.GetRenderingDevice();	// Get the global RD since there's a lot of GPU side stuff that needs it
	private GpuPlanet gpuPlanet;
	private Dictionary<string, Rid> Buffers = new Dictionary<string, Rid>();
	private Dictionary<string, ShaderInfo> Shaders = new Dictionary<string, ShaderInfo>();
	private Rid LayoutSet;

	Rid LoadPipeline(string path, out Rid shader)
	{
		// Get the loaded shader from the path and return the pipeline RID
		var spirv = GD.Load<RDShaderFile>(path).GetSpirV();
		shader = rd.ShaderCreateFromSpirV(spirv);
		return rd.ComputePipelineCreate(shader);
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{

		// Get multimesh and tranform information setup
		multiMesh = GetNode<MultiMeshInstance3D>("MultiMeshInstance3D").Multimesh;
		multiMesh.TransformFormat = MultiMesh.TransformFormatEnum.Transform3D;

		// Get partition info at runtime
		float partitionSize = 2f * Mathf.Max(SeparationRadius, Mathf.Max(AlignmentRadius, CohesionRadius));
		uint partsPerAxis = (uint) Mathf.Ceil(2f*Planet.Radius/partitionSize);
		uint numPartitions = (uint) Mathf.Pow(partsPerAxis, 3);

		// Set planet info. Eventually gonna be per planet scene on creation when it's precedural
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

		// Generate the boids and set their info so the GPU doesn't have to worry about it
		Boids = new GpuBoid[SchoolCounts.Sum()];
		// Generate a bunch of random starting positions and rotations around the planet
		RandomNumberGenerator gpuRng = new RandomNumberGenerator();
		int SchoolSum = 0;
		for (uint i = 0; i < SchoolCounts.Length; i++)
		{
			for (uint j = 0; j < SchoolCounts[i]; j++)
			{
				Vector3 dir = new Vector3(gpuRng.RandfRange(-3.1f, 3.1f), gpuRng.RandfRange(-3.1f, 3.1f), gpuRng.RandfRange(-3.1f, 3.1f)).Normalized();	// Random rotation to pick locations for Boids
				Vector3 position = Planet.GlobalPosition + dir * (Planet.Scale.X*0.5f + gpuRng.RandfRange(SpawnHeight[0], SpawnHeight[1]));
				Boids[i+j] = new GpuBoid
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
			SchoolSum += SchoolCounts[i];
		}

		// The flattened view of the gpu boids which will be used to pass to the GPU as the boids array
		multiMesh.InstanceCount = Boids.Length;	// All the boids exist in the meshinstance now
		Godot.Collections.Array<RDUniform> uniforms = new Godot.Collections.Array<RDUniform>();

		byte[] boidBytes = MemoryMarshal.AsBytes(Boids.AsSpan()).ToArray();
		// Create and get buffer ID
		Rid boidBuffer = rd.StorageBufferCreate((uint)boidBytes.Length, boidBytes);
		Buffers.Add("Boids", boidBuffer);
		var boidUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.StorageBuffer,
			Binding = 0
		};
		boidUniform.AddId(boidBuffer);
		uniforms.Append(boidUniform);

		// Create the partCounts buffer, which doesn't get read until it's populated by GPU, so no worries on initial data
		Rid partCountBuffer = rd.StorageBufferCreate(numPartitions*4u);	// * 4u because we need number of bytes and uint is 4 bytes
		Buffers.Add("PartCount", partCountBuffer);
		var partCountUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.StorageBuffer,
			Binding = 1
		};
		partCountUniform.AddId(partCountBuffer);
		uniforms.Append(partCountUniform);

		// Create sorted boids secondary buffer, again no worries about init data, it gets written to by GPU before reading
		Rid sortedBoidsBuffer = rd.StorageBufferCreate((uint)(Boids.Length * Unsafe.SizeOf<GpuBoid>()));
		Buffers.Add("SortedBoids", sortedBoidsBuffer);
		var sortedBoidsUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.StorageBuffer,
			Binding = 2
		};
		sortedBoidsUniform.AddId(sortedBoidsBuffer);
		uniforms.Append(sortedBoidsUniform);

		// Create Planet on GPU. This will change each time the player moves to another planet
		byte[] planetBytes = MemoryMarshal.AsBytes(MemoryMarshal.CreateSpan(ref gpuPlanet, 1)).ToArray();	// Get length in bytes
		Rid planetBuffer = rd.UniformBufferCreate((uint)planetBytes.Length, planetBytes);
		Buffers.Add("Planet", planetBuffer);
		var planetUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.UniformBuffer,
			Binding = 3
		};
		planetUniform.AddId(planetBuffer);
		uniforms.Append(planetUniform);

		// Create partition index buffer, for storing starting index of partition in the sorted boids buffer
		Rid partIdxBuffer = rd.StorageBufferCreate((uint)(numPartitions * 4u));
		Buffers.Add("PartIdx", partIdxBuffer);
		var partIdxUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.StorageBuffer,
			Binding = 4
		};
		partIdxUniform.AddId(partIdxBuffer);
		uniforms.Append(partIdxUniform);

		// Create schoolCounts buffer
		byte[] schoolCountBytes = MemoryMarshal.AsBytes(SchoolCounts.AsSpan()).ToArray();
		Rid schoolCountsBuffer = rd.StorageBufferCreate((uint)schoolCountBytes.Length, schoolCountBytes);
		Buffers.Add("SchoolCounts", schoolCountsBuffer);
		var schoolCountsUniform = new RDUniform
		{
			UniformType = RenderingDevice.UniformType.StorageBuffer,
			Binding = 5
		};
		schoolCountsUniform.AddId(schoolCountsBuffer);
		uniforms.Append(schoolCountsUniform);

		Shaders.Add("Layout", new ShaderInfo(rd, "res://scripts/GLSL/layout.glsl"));
		Shaders.Add("Partition", new ShaderInfo(rd, "res://scripts/GLSL/partition.glsl"));
		Shaders.Add("Count", new ShaderInfo(rd, "res://scripts/GLSL/count_parts.glsl"));
		Shaders.Add("Sum", new ShaderInfo(rd, "res://scripts/GLSL/sum_parts.glsl"));
		Shaders.Add("Rules", new ShaderInfo(rd, "res://scripts/GLSL/boid_rules.glsl"));

		// Finally put them all in set 0 in the common layout then bind to each shader
		LayoutSet = rd.UniformSetCreate(uniforms, Shaders["Layout"].Shader, 0);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	public override void _PhysicsProcess(double delta)
	{
		float dt = (float)delta;
		var world = GetWorld3D();

		long list = rd.ComputeListBegin();

		foreach (ShaderInfo val in Shaders.Values)
		{
			
		}

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
