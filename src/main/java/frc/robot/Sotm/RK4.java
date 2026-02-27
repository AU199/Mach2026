package frc.robot.Sotm;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

/**
 * Runge-Kutta 4th order integrator for ball trajectory.
 * 
 * Models drag, Magnus (spin) force, and gravity.
 * Angular velocity is assumed constant throughout flight (reasonable for short FRC shots).
 * 
 * Coordinate system: field-relative, X = field length axis, Y = field width axis, Z = up.
 */
public class RK4 {

    // ── Physical constants ──────────────────────────────────────────────────
    private static final double AIR_DENSITY     = 1.2250;                                   // kg/m^3 at sea level
    private static final double DRAG_COEFF      = 0.46;                                     // dimensionless, smooth sphere
    private static final double LIFT_COEFF      = 0.5;                                      // Magnus lift coefficient
    private static final double BALL_RADIUS_IN  = 5.91 / 2.0;                               // inches
    private static final double BALL_AREA       = Math.PI * Math.pow(BALL_RADIUS_IN * 0.0254, 2); // m^2
    private static final double BALL_MASS_KG    = Kilograms.convertFrom(0.5, Pounds);
    private static final double GRAVITY         = 9.81;                                     // m/s^2

    // ½ρA·C_L — scalar factor for Magnus force
    private static final double MAGNUS_COEFF = 0.5 * AIR_DENSITY * BALL_AREA * LIFT_COEFF;

    // ── Simulation state ────────────────────────────────────────────────────
    private final double dt;
    private final Pose2d targetPose;
    private final Pose2d shooterPose;
    private final Vector<N3> ballInitialAngularVelocity; // field-relative, constant throughout flight

    // ── Constructor ─────────────────────────────────────────────────────────

    /**
     * @param targetPose                             Field-relative target (hub center projected to field plane)
     * @param shooterPose                            Field-relative shooter exit position (x, y)
     * @param ballInitialLinearVelocityRelativeToField  Initial ball velocity in field frame (m/s)
     * @param ballInitialAngularVelocityRelativeToField Ball spin angular velocity in field frame (rad/s)
     * @param dt                                     Integration timestep (seconds, typically 0.001)
     */
    public RK4(
            Pose2d targetPose,
            Pose2d shooterPose,
            ChassisSpeeds robotFieldRelativeVelocity,       // kept for API compatibility, robot vel already baked into ball vel
            Vector<N3> ballInitialLinearVelocityRelativeToField,
            Vector<N3> ballInitialAngularVelocityRelativeToField,
            Pose2d ballInitialPose,                         // unused — ball starts at shooterPose + shooterHeight
            double dt) {
        this.targetPose = targetPose;
        this.shooterPose = shooterPose;
        this.ballInitialAngularVelocity = ballInitialAngularVelocityRelativeToField;
        this.dt = dt;
    }

    // ── Cross product (not in WPILib Vector) ────────────────────────────────

    private Vector<N3> cross(Vector<N3> a, Vector<N3> b) {
        return VecBuilder.fill(
            a.get(1) * b.get(2) - a.get(2) * b.get(1),
            a.get(2) * b.get(0) - a.get(0) * b.get(2),
            a.get(0) * b.get(1) - a.get(1) * b.get(0)
        );
    }

    // ── Force / acceleration helpers ────────────────────────────────────────

    /**
     * Aerodynamic drag: F = -½ρ·Cd·A·|v|·v  →  a = F/m
     * Direction opposes velocity, magnitude scales with v².
     */
    private Vector<N3> dragAcceleration(Vector<N3> v) {
        double speed = v.norm();
        if (speed < 1e-9) return VecBuilder.fill(0, 0, 0);
        double scalar = -0.5 * AIR_DENSITY * DRAG_COEFF * BALL_AREA * speed / BALL_MASS_KG;
        return VecBuilder.fill(0, 0, 0); // Temporary for now
        // return v.times(scalar);
    }

    /**
     * Magnus (spin) force: F = C_L · ½ρA · (ω × v)  →  a = F/m
     * Uses constant initial angular velocity (valid approximation for short shots).
     */
    private Vector<N3> magnusAcceleration(Vector<N3> v) {
        Vector<N3> omegaCrossV = cross(ballInitialAngularVelocity, v);
        double scalar = MAGNUS_COEFF / BALL_MASS_KG;
        return VecBuilder.fill(0, 0, 0); // Temporary for now
        // return omegaCrossV.times(scalar);
    }

    /** Gravitational acceleration (constant, downward). */
    private Vector<N3> gravityAcceleration() {
        return VecBuilder.fill(0, 0, -GRAVITY);
    }

    /** Total acceleration = drag + Magnus + gravity. */
    private Vector<N3> totalAcceleration(Vector<N3> v) {
        return dragAcceleration(v)
            .plus(magnusAcceleration(v))
            .plus(gravityAcceleration());
    }

    // ── RK4 step ────────────────────────────────────────────────────────────

    /**
     * Advances position and velocity by one timestep dt using RK4.
     */
    private BallState rk4Step(Vector<N3> pos, Vector<N3> vel) {
        // k's are acceleration (dv/dt), m's are velocity (dx/dt)
        Vector<N3> m1 = vel;
        Vector<N3> k1 = totalAcceleration(vel);

        Vector<N3> m2 = vel.plus(k1.times(dt / 2.0));
        Vector<N3> k2 = totalAcceleration(m2);

        Vector<N3> m3 = vel.plus(k2.times(dt / 2.0));
        Vector<N3> k3 = totalAcceleration(m3);

        Vector<N3> m4 = vel.plus(k3.times(dt));
        Vector<N3> k4 = totalAcceleration(m4);

        Vector<N3> newPos = pos.plus(
            (m1.plus(m2.times(2.0)).plus(m3.times(2.0)).plus(m4)).times(dt / 6.0)
        );
        Vector<N3> newVel = vel.plus(
            (k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4)).times(dt / 6.0)
        );

        return new BallState(newPos, newVel);
    }

    // ── Main trajectory integration ─────────────────────────────────────────

    /**
     * Integrates the ball trajectory from the shooter until the ball descends back
     * through hub height, then returns the (x, y) error vs the target.
     *
     * Returns NaN errors if:
     *   - the ball hits the ground before clearing hub height
     *   - integration exceeds maxSteps without the ball descending through hub height
     */
    public BallError calculateError(Vector<N3> ballInitialLinearVelocity) {
        Vector<N3> position = VecBuilder.fill(
            shooterPose.getX(),
            shooterPose.getY(),
            Constants.shooterHeight
        );
        Vector<N3> velocity = ballInitialLinearVelocity;

        boolean hasGoneAboveHub = false;
        final int maxSteps = 5000;

        for (int step = 0; step < maxSteps; step++) {
            double z = position.get(2);

            // Track whether ball has cleared hub height
            if (z >= Constants.hubZ) {
                hasGoneAboveHub = true;
            }

            // Ball has gone above hub and is now descending back through hub height — done
            if (hasGoneAboveHub && z < Constants.hubZ) {
                break;
            }

            // Ball hit the ground before clearing hub — bad trajectory
            if (z < 0.0) {
                return new BallError(Double.NaN, Double.NaN);
            }

            BallState next = rk4Step(position, velocity);
            position = next.getPosition();
            velocity = next.getVelocity();
        }

        // If we exhausted steps without the ball descending through hub height
        if (!hasGoneAboveHub || position.get(2) >= Constants.hubZ) {
            System.out.println("ERROR THE BALL DID NOT GO OVER. DO NOT REDEEM");
            return new BallError(Double.NaN, Double.NaN);
        }

        double xError = targetPose.getX() - position.get(0);
        double yError = targetPose.getY() - position.get(1);
        return new BallError(xError, yError);
    }
}
