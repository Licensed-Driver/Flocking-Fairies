using Godot;
using System;
using System.Net.Http;

public partial class Player : CharacterBody3D
{
	[Export] public float MouseSens = 30;
	[Export] public float Gravity = -9.81f;
	[Export] public Node3D Planet;


	[Export] public float Speed = 5.0f;
	[Export] public float Accel = 5.0f;
	[Export] public float Decel = 5.0f;
	[Export] public float JumpVelocity = 4.5f;

	private Node3D head;
	private float headPitch = 0.0f;

	public override void _Ready()
	{
		base._Ready();

		head = GetNode<Node3D>("Head");
		Input.MouseMode = Input.MouseModeEnum.Captured;
	}

	public override void _UnhandledInput(InputEvent e)
	{
		base._UnhandledInput(e);

		if (e is InputEventMouseMotion m)
		{
			// Rotate the body along Y axis when mouse moves along X
			RotateObjectLocal(Vector3.Up, -m.Relative.X * MouseSens/1000); // Divide by 1000 so we can have reasonable sclae in inspector
			// Clamp pitch between +/-1.5 because pi/2 and -pi/2 are like +/-1.5something so we would line up with the axis and have fuck shit go on
			headPitch = Mathf.Clamp(headPitch - m.Relative.Y * MouseSens/1000, -1.5f, 1.5f);
			head.Rotation = new Vector3(headPitch, 0, 0);
		}

		if (e.IsActionPressed("ui_cancel")) // Escape by default
			Input.MouseMode = Input.MouseModeEnum.Visible;

		if (e is InputEventMouseButton b && b.Pressed)
			Input.MouseMode = Input.MouseModeEnum.Captured;
	}

	public override void _PhysicsProcess(double delta)
	{
		float dt = (float)delta;

		// Update up direction before movement
		Vector3 up = Planet.GlobalPosition.DirectionTo(GlobalPosition);
		UpDirection = up;

		// Set new basis so all the future vector math is updated
		Vector3 forward = (-GlobalBasis.Z).Slide(up).Normalized();
		Vector3 right = forward.Cross(up).Normalized();
		GlobalBasis = new Basis(right, up, -forward);

		// Get the vertical by projecting onto our relative up, then get all components that aren't in up direction
		Vector3 vertical = Velocity.Project(up);
		Vector3 horizontal = Velocity.Slide(up);

		// Jump and Gravity
		if (IsOnFloor())
		{
			vertical = -up * dt;	// Tiny little down pressure while we are on the ground so we maintain reasonable contact
			
		} else
		{
			//vertical += Gravity * up * dt;
		}
		if (Input.IsActionJustPressed("ui_accept"))
			{
				vertical += JumpVelocity * up;
			}
		if (Input.IsActionJustPressed("sprint"))
		{
			vertical -= JumpVelocity * up;
		}

		// Get the input direction and handle the movement/deceleration.
		// As good practice, you should replace UI actions with custom gameplay actions.
		Vector2 inputDir = Input.GetVector("move_left", "move_right", "move_forward", "move_backward");
		Vector3 direction = (GlobalBasis * new Vector3(inputDir.X, 0, inputDir.Y)).Normalized();
		
		// Horizontal movement
		Vector3 target = direction * Speed;
		float rate = direction != Vector3.Zero && horizontal.Dot(direction) >= 0 ? Accel : Decel;	// If user is moving and in the direction of their velocity then chill, otherwise we wanna decelerate
		horizontal = horizontal.MoveToward(target, rate * dt);

		Velocity = vertical + horizontal;
		MoveAndSlide();
	}
}
