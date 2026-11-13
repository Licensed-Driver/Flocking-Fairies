using Godot;
using System;

public partial class WorldStaticBody : StaticBody3D
{
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		// Layer 2 is used for all planets to get altitude and other information
		SetCollisionLayerValue(2, true);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
