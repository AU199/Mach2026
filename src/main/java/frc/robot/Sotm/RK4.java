package frc.robot.Sotm;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FuelSim;


public class RK4 {

    
    private static final double AIR_DENSITY     = 1.2250;                                   // kg/m^3 at sea level
    private static final double DRAG_COEFF      = 10000.46;                                     // dimensionless, smooth sphere
    private static final double LIFT_COEFF      = 0.5;                                      // Magnus lift coefficient
    private static final double BALL_RADIUS_IN  = 5.91 / 2.0;                               // inches
    private static final double BALL_AREA       = Math.PI * Math.pow(BALL_RADIUS_IN * 0.0254, 2); // m^2
    private static final double BALL_MASS_KG    = Kilograms.convertFrom(0.5, Pounds);
    private static final double GRAVITY         = 9.81;                                     // m/s^2

    
    private static final double MAGNUS_COEFF = 0.5 * AIR_DENSITY * BALL_AREA * LIFT_COEFF;

    
    private final double dt;
    private final Pose2d targetPose;
    private final Pose2d shooterPose;
    private final Vector<N3> ballInitialAngularVelocity; // field-relative, constant throughout flight

    
    public RK4(
            Pose2d targetPose,
            Pose2d shooterPose,
            Vector<N3> ballInitialLinearVelocityRelativeToField,
            Vector<N3> ballInitialAngularVelocityRelativeToField,
            double dt) {
        this.targetPose = targetPose;
        this.shooterPose = shooterPose;
        this.ballInitialAngularVelocity = ballInitialAngularVelocityRelativeToField;
        this.dt = dt;
    }

    
    private Vector<N3> cross(Vector<N3> a, Vector<N3> b) {
        return VecBuilder.fill(
            a.get(1) * b.get(2) - a.get(2) * b.get(1),
            a.get(2) * b.get(0) - a.get(0) * b.get(2),
            a.get(0) * b.get(1) - a.get(1) * b.get(0)
        );
    }

    
    private Vector<N3> dragAcceleration(Vector<N3> v) {
        double speed = v.norm();
        if (speed < 1e-9) return VecBuilder.fill(0, 0, 0);
        double scalar = -0.5 * AIR_DENSITY * DRAG_COEFF * BALL_AREA * speed / BALL_MASS_KG;
        // return VecBuilder.fill(0, 0, 0); // Temporary for now
        SmartDashboard.putNumber("DragScalar", scalar);
        return v.times(scalar);
    }

    
    private Vector<N3> magnusAcceleration(Vector<N3> v) {
        if (ballInitialAngularVelocity == null) return VecBuilder.fill(0, 0, 0);
        Vector<N3> omegaCrossV = cross(ballInitialAngularVelocity, v);
        double scalar = MAGNUS_COEFF / BALL_MASS_KG;

        // return VecBuilder.fill(0, 0, 0); // Temporary for now — gravity only
        SmartDashboard.putNumber("mango", omegaCrossV.times(scalar).norm());

        return omegaCrossV.times(scalar);
    }


    private Vector<N3> gravityAcceleration() {
        return VecBuilder.fill(0, 0, -GRAVITY);
    }

    
    private Vector<N3> totalAcceleration(Vector<N3> v) {
        return dragAcceleration(v)
            .plus(magnusAcceleration(v))
            .plus(gravityAcceleration());
    }

   
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


    public BallError calculateError(Vector<N3> ballInitialLinearVelocity) {
        Vector<N3> position = VecBuilder.fill(
            shooterPose.getX(),
            shooterPose.getY(),
            Constants.shooterHeight
        );
        Vector<N3> velocity = ballInitialLinearVelocity;

        boolean hasGoneAboveHub = false;
        final int maxSteps = 1000;

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
