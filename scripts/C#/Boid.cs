using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using Godot;

public class Boid
{
    private float Radius;	// Radius from the center of its planet
    private float separationRad;
    private float alignmentRad;
    private float cohesionRad;
    private float separationCoef = 1.0f;
    private float alignmentCoef = 1.0f;
    private float cohesionCoef = 1.0f;
    private Node3D planet;
    private Vector3 planetPosition;	// Which planet this boid belongs to
    private Vector3 desiredHeading;	// Acceleration to apply realistic boid rules over time
    private float altitude;
    private float ground;   // Ground level

    public Vector3 Velocity;	// Position, velocity, and rotation in global space
    public Basis VelocityBasis;	// From local space to global for computing our movement
    public Basis RotationBasis;	// From local space to global, doesn't effect movement, only mesh orientation

    public Vector3 Position;
    public Vector3 GlobalPosition;
    public float FieldOfView;

    public float TurnRate;
    public float MaxAcceleration;	// Maximum length of the acceleration vector per second
    // Max altitude is relative to the ground height, max radius is pure distance from planet centre
    public float MaxAlt;
    public float MaxRad;

    /*
    Create new Boid with initial position, heading, and belonging to planet.
    Heading is used as a direction to face on the sphere, but up with always initialize as orthogonal to the planet surface
    */
    public Boid(Vector3 position, Node3D planet, Vector3 heading, float maxAlt = 0, float turnRate = Mathf.Pi/8.0f, float fieldOfView = 90.0f)
    {
        GlobalPosition = position;
        planetPosition = planet.GetGlobalPosition();
        Position = GlobalPosition - planetPosition;
        Radius = planet.GlobalPosition.DistanceTo(position);
        MaxAlt = maxAlt > 0 ? maxAlt : 1.5f * Radius;
        this.planet = planet;

        RotationBasis = createPlanetRelativeBasis(heading);
        Vector3 up = planetPosition.DirectionTo(GlobalPosition);
        Vector3 facing = RotationBasis.Y.Slide(up);
        Vector3 forward = facing.LengthSquared() > 1e-6f
            ? facing.Normalized()
            : (-VelocityBasis.Z).Slide(up).Normalized();
        Vector3 right = forward.Cross(up).Normalized();
        VelocityBasis = new Basis(right, up, -forward);

        FieldOfView = fieldOfView;
        TurnRate = turnRate;

        // Normal distribution centered around half the radius so comfort scales with how close or far within radius
    }

    private Basis createPlanetRelativeBasis(Vector3 heading)
    {
        // Take standard Basis, rotate it from North Pole (only point on a planet where Basis is standard) to the actual position
        // Use that rotated basis to convert their heading to planet relative, then create the rotation basis on that
        Vector3 northPole = Vector3.Up * Radius;
        Basis planetBasis = new Basis(northPole.Cross(Position).Normalized(), northPole.AngleTo(Position));

        Vector3 up = Position.Normalized();

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

    public void SetRuleRadii(float separationRad, float alignmentRad, float cohesionRad)
    {
        this.separationRad = separationRad;
        this.alignmentRad = alignmentRad;
        this.cohesionRad = cohesionRad;
    }

    public void SetRuleCoefficients(float separationCoef, float alignmentCoef, float cohesionCoef)
    {
        this.separationCoef = separationCoef;
        this.alignmentCoef = alignmentCoef;
        this.cohesionCoef = cohesionCoef;
    }

    public void MainThreadTasks(World3D world, PhysicsRayQueryParameters3D query)
    {
        checkAltitude(world, query);
    }

    private void checkAltitude(World3D world, PhysicsRayQueryParameters3D query)
    {
        // Cast planet space down by radius to cast to centre of planet
        // Bound to hit the planet eventually so we just keep using radius
        var castLength = (-VelocityBasis.Y * Radius);
        query.From = GlobalPosition;
        query.To = GlobalPosition + castLength;
        var results = world.DirectSpaceState.IntersectRay(query);
        // Mathematically guaranteed to hit the planet, but fallback to -1 so we can signal to hold steady.
        if (results.Count > 0)
        {
            altitude = (GlobalPosition - (Vector3)results["position"]).Length();
        }
        else altitude = -1;

        ground = Radius - altitude;
    }

    /*
    Cheap check for obstacles in front of a boid.
    Cast as many rays evenly distributed around the circle projected in front of the boid as given.
    Adds padding to projected circle, stretching it out to bound boid plus padding, with Z being distance from boid.
    */
    private void checkObstacles(World3D world, PhysicsRayQueryParameters3D query, int points, Vector3 offset)
    {
        float angle;
        float anglePerPoint = (2 * Mathf.Pi)/points;
        query.From = GlobalPosition;
        Vector3 to = GlobalPosition;
        for (int i = 0; i < points; i++)
        {
            angle = i * anglePerPoint;
            query.To = GlobalPosition;

        }
        

    }

    private void processRules(Boid[] boids)
    {
        // Separation, alignment, and cohesion acceleration vectors
        Vector3 sVec = Vector3.Zero;
        Vector3 aVec = Vector3.Zero;
        Vector3 cVec = Vector3.Zero;
        int aCount = 0;
        int cCount = 0;

        for (int i = 0; i < boids.Length; i++) {
            if (boids[i] == this) continue;
            // Vector3 struct so it's copied, so we do it once
            float distanceTo;
            // Great circle direction
            Vector3 sphericalDirection;
            // Only process if we can see the other boid
            if (RotationBasis.Y.AngleTo(boids[i].Position - Position) > FieldOfView/2.0f) continue;
            // Using our own radius since we account for altitude later
            float arclength = Position.AngleTo(boids[i].Position) * Radius;
            if (arclength < 1e-8)	// Other boid is directly above or below on this side of planet
            {
                distanceTo = (boids[i].Position - Position).Length();
                sphericalDirection = boids[i].Position.Normalized();
            } else	// Other boid is other side of planet
            {
                distanceTo = Mathf.Sqrt(arclength*arclength + (boids[i].Radius - Radius)*(boids[i].Radius - Radius));
                sphericalDirection = (boids[i].Position.Slide(VelocityBasis.Y).Normalized() * arclength + VelocityBasis.Y * (boids[i].Radius - Radius)).Normalized();
            }

            // All guaranteed normal or normalized
            if (distanceTo <= separationRad)
            {
                sVec -= sphericalDirection / (distanceTo*distanceTo);
            }
            if (distanceTo <= alignmentRad)
            {
                aCount++;
                // Their pitch relative to their own surface
                float sinPitch = boids[i].RotationBasis.Y.Dot(boids[i].VelocityBasis.Y);
                float cosPitch = boids[i].RotationBasis.Y.Slide(boids[i].VelocityBasis.Y).Length();
                // Get how where they are going forward in our tangent plane
                Vector3 forwardRelToUs = (-boids[i].VelocityBasis.Z).Slide(VelocityBasis.Y);
                if (forwardRelToUs.LengthSquared() >= 1e-8) forwardRelToUs = forwardRelToUs.Normalized() * cosPitch;
                // Get how much they are going up in their plane
                forwardRelToUs += VelocityBasis.Y * sinPitch;
                aVec += forwardRelToUs;
            }
            if (distanceTo <= cohesionRad)
            {
                cCount++;
                cVec += sphericalDirection * distanceTo;
            }
        };
        Vector3 steer = sVec * VelocityBasis * separationCoef;
        if (aCount > 0) steer += (aVec/aCount) * VelocityBasis * alignmentCoef;
        if (cCount > 0) steer += ((cVec/cCount) * VelocityBasis).Normalized() * cohesionCoef;
        // add em up
        desiredHeading = steer.LengthSquared() > 1e-6f ? steer : RotationBasis.Y * VelocityBasis;
        if (altitude < 0)   // Raycast missed, so we just negate any climb or descent
        {
            desiredHeading = desiredHeading.Slide(Vector3.Up);
        }
        // Planet space so Y is local up
        // Velocity.Y is forward rn, also technically boids could translate up/down and side-to-side out of altitude, but they never do those so this works until they do
        if (desiredHeading.Normalized().Y * Velocity.Y + altitude > MaxAlt)  // If we go above maxalt we correct by forcing local down
        {
            desiredHeading.Y = 0;
            desiredHeading = desiredHeading.Normalized();
            desiredHeading.Y = -1;
        }
        if (desiredHeading.Normalized().Y * Velocity.Y + altitude < 0)
        {
            desiredHeading.Y = 0;
            desiredHeading = desiredHeading.Normalized();
            desiredHeading.Y = 1;
        }
    }

    // Discomfort from -1 to 1 where -1 means move away and 1 means move closer
    //	Little fairies get stressed out when people get too close or too far
    // Good values are skew=0.25 and deadZone=min(m, r-m)/2
    private float discomfort(float x, float skew, float searchRadius)
    {
        skew = Mathf.Clamp(skew, 0.01f, 0.99f);
        float mean = skew * searchRadius;
        float deadZone = 0.0f;
        deadZone = Mathf.Clamp(deadZone, 0f, 0.99f * Mathf.Min(mean, searchRadius - mean));
        x = Mathf.Clamp(x, 0f, searchRadius);

        float span = x < mean ? mean - deadZone : searchRadius - mean - deadZone;   // distance from dead zone edge to radius edge, on this side
        float d = Mathf.Max(Mathf.Abs(x - mean) - deadZone, 0f);
        return Mathf.Sign(x - mean) * (d * d) / (span * span);
    }

    // Checks if the boid is within their vision cone
    private bool canSeeBoid(Boid other)
    {
        return RotationBasis.Y.AngleTo(other.GlobalPosition - GlobalPosition) < Mathf.DegToRad(FieldOfView)/2.0f;
    }

    public void LookAt(Vector3 heading, float delta)
    {
        Vector3 current = RotationBasis.Y;
        Vector3 target = VelocityBasis * heading.Normalized();

        Vector3 axis = current.Cross(target);
        if (axis.LengthSquared() > 1e-8)
        {
            axis = axis.Normalized();
            float angle = current.AngleTo(target) * Mathf.Clamp(delta * TurnRate, 0.0f, 1.0f);

            if (angle > 1e-8f)	// Don't do micro updates
                RotationBasis = (new Basis(axis, angle) * RotationBasis).Orthonormalized();	// Currect any tiny lil errors so we don't drift
        }
    }

    public void Simulate(float delta)
    {
        // Calculate now so we can do math, then update later for after movement basis
        Vector3 up = planetPosition.DirectionTo(GlobalPosition);

        Vector3 worldVelocity = RotationBasis * Velocity;	// Go from rotational to global

        // We need to rotate around the planet for X and Z movement
        Vector3 horizontal = worldVelocity.Slide(up);
        Vector3 axis = up.Cross(horizontal).Normalized();	// In global space so use global version of planet up
        float angle = horizontal.Length() * delta/Radius;	// Radians are arclength/radius

        // Boost out first since up isn't correct after the rotation, and the rotation will carry it through
        float comp = worldVelocity.Dot(up) * delta;
        GlobalPosition += up * comp;
        Position = GlobalPosition - planetPosition;
        Radius = (GlobalPosition - planetPosition).Length();

        // Basis is used later anyways so better to just build a rotation basis instead of the simplified rodrigues
        Basis rodrigues = new Basis(axis, angle);
        // Planet space not global, otherwise we'd rotate around Vector3.Zero
        GlobalPosition = planetPosition + rodrigues * (GlobalPosition - planetPosition);

        // Keep both bases in sync since they are always at the same position
        // rodrigues was built in planet relative velocity space, but RotationBasis is in global space
        RotationBasis = (rodrigues * RotationBasis).Orthonormalized();

        // To ensure that float rounding doesn't ever shift us off radial
        up = planetPosition.DirectionTo(GlobalPosition);
        Vector3 facing = RotationBasis.Y.Slide(up);
        Vector3 forward = facing.LengthSquared() > 1e-6f
            ? facing.Normalized()
            : (-VelocityBasis.Z).Slide(up).Normalized();
        Vector3 right = forward.Cross(up).Normalized();
        VelocityBasis = new Basis(right, up, -forward);
    }

    public void ProcessBoids (Boid[] boids)
    {
        // We need to 1. Maintian good distance from flockmates 2. Steer to fly with flockmates 3. keep with the flock
        // My idea is using a normal distribution centered around a "Comfort Mean", Where being a little closer or further than like 1 is fine, but we try more and more to get closer or further when we break from that mean. Make it wide like std = 2 or something
        /*float[] discomfort = new float[boids.Length];
        float expSums = 0.0f;
        int valid = 0;
        Vector3 averageHeading = Vector3.Zero;
        Vector3 weightedVel = Vector3.Zero;

        // We move toward vector to each boid weighted by discomfort and normalized
        for (int i = 0; i < boids.Length; i++)
        {
            float distance = boids[i].Position.DistanceTo(Position);
            if (boids[i] == this || distance > radius || !canSeeBoid(boids[i])) continue;
            valid++;
            Vector3 target = Vector3.Zero;
            if (distance < radius/4)
            {
                
            }
            // Softmax magnitudes but preserve direction
            float dis = this.discomfort(boids[i].Position.DistanceTo(Position), 0.25f, radius);
            discomfort[i] = Mathf.Sign(dis) * Mathf.Exp(+Mathf.Abs(dis)/neighbourStrength);
            expSums += Mathf.Exp(+Mathf.Abs(dis)/neighbourStrength);
            averageHeading += boids[i].RotationBasis.Y;
        }

        // Nothing within range
        if (valid == 0) return;
        averageHeading /= valid;

        for (int i = 0; i < boids.Length; i++)
        {
            float distance = boids[i].Position.DistanceTo(Position);
            if (boids[i] == this || distance > radius) continue;
            // Weighted velocity away or towards
            weightedVel += (boids[i].Position - Position).Normalized() * (discomfort[i]/expSums);
        }

        LookAt(Position + (averageHeading + weightedVel)/2.0f, 5*delta);
        */

        processRules(boids);
    }

    public void ApplyHeading (float delta)
    {
        LookAt(desiredHeading, delta);
    }
}