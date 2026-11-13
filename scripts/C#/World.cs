using Godot;
using System;

public partial class World : MeshInstance3D
{
	[Export] public float Radius;
	[Export] public float Gravity;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		Transform.Scaled(Vector3.One * 2 * Radius);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		
	}

	public Basis CreateRelativeBasis(Vector3 position, Vector3 heading)
	{
		// Take standard Basis, rotate it from North Pole (only point on a planet where Basis is standard) to the actual position
		// Use that rotated basis to convert their heading to planet relative, then create the rotation basis on that
		Vector3 northPole = Vector3.Up * Radius;
		Vector3 relPosition = position - Position;
		Basis planetBasis = new Basis(northPole.Cross(relPosition).Normalized(), northPole.AngleTo(relPosition));

		Vector3 up = Position.DirectionTo(position);

		// Rotate heading to be planet relative instead of global
		heading = planetBasis * heading;

		if (Mathf.Abs(heading.Cross(up).Length()) < 1e-3)	// Straight up so we just grab an arbitrary vector perp to heading
		{
			Vector3 axis = planetBasis * Vector3.Forward;
			up = heading.Cross(axis).Normalized();
		} else
		{
			up = up.Slide(heading).Normalized();
		}
		Vector3 right = heading.Cross(up).Normalized();
		return new Basis(right, heading, up);
	}

	public Vector3 Global2Spherical(Vector3 position)
	{
		Vector3 relPosition = Position - position;
		float radius = relPosition.Length();

		return new Vector3(radius, Mathf.Acos(relPosition.Z/radius), Mathf.Sign(relPosition.Y) * Mathf.Acos(relPosition.X/Mathf.Sqrt(relPosition.X*relPosition.X+relPosition.Y*relPosition.Y)));
	}

	public Vector3 Spherical2Global(Vector3 spherePosition)
	{
		float x = spherePosition.X*Mathf.Sin(spherePosition.Y)*Mathf.Cos(spherePosition.Z);
		float y = spherePosition.X*Mathf.Sin(spherePosition.Y)*Mathf.Sin(spherePosition.Z);
		float z = spherePosition.X*Mathf.Cos(spherePosition.Y);
		return new Vector3(x, y, z) + Position;
	}

	public float SphericalDistance(Vector3 from, Vector3 to)
	{
		return (Spherical2Global(to) - Spherical2Global(from)).Length();
	}
}
