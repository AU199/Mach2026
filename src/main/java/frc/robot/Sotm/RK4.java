package frc.robot.Sotm;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class RK4 {
    private double standardFluidDensityOfAir = 1.2250;
    private double areaOfBall = Math.PI * Math.pow(5.91 / 2 * 0.0762, 2);
    private double dragCoefficient = 0.46;
    private double liftCoefficient = 0.5;
    private double carlsenCoefficient = (1 / 2) * standardFluidDensityOfAir * areaOfBall * liftCoefficient;

    private double dt;

    private double xError;
    private double yError;

    private Pose2d targetPose;
    private Pose2d shooterPose;

    private double ballMass = Kilograms.convertFrom(0.5, Pounds);
    private double accelerationOfGravity = 9.81;

    private Vector ballInitialLinearVelocity;
    private Vector ballInitialAngularVelocity;

    private BallState ballState;

    private Vector calculateDragAcceleration(Vector ballLinearVelocity) {
        Vector dragForce = ballLinearVelocity
            .times((-1 / 2) * standardFluidDensityOfAir * dragCoefficient * areaOfBall * ballLinearVelocity.norm());
        Vector dragAcceleration = dragForce.div(ballMass);
        return dragAcceleration;
    }

    private Vector calculateMagnusAcceleration(Vector linearVelocity) {
        Vector carlsenForce = Vector.cross(ballInitialAngularVelocity, linearVelocity).times(carlsenCoefficient);
        Vector carlsenAcceleration = carlsenForce.div(ballMass);
        return carlsenAcceleration;
    }

    private Vector calculateGravityAcceleration() {
        Vector gravityAcceleration = VecBuilder.fill(0, 0, -accelerationOfGravity);
        return gravityAcceleration;
    }

    public RK4(Pose2d targetPose, ChassisSpeeds robotVelocity,
            Vector ballInitialLinearVelocityRelativeToField, Vector ballInitialAngularVelocityRelativeToField,
            Pose2d ballInitialPose, double dt) {
        this.targetPose = targetPose;

        this.dt = dt;

        this.ballInitialLinearVelocity = ballInitialLinearVelocityRelativeToField;
        this.ballInitialAngularVelocity = ballInitialAngularVelocityRelativeToField;
    }

    private Vector calculateAcceleration(Vector linearVelocity) {
        Vector dragAcceleration = calculateDragAcceleration(linearVelocity);
        Vector magnusAcceleration = calculateMagnusAcceleration(linearVelocity);
        Vector gravityAcceleration = calculateGravityAcceleration();

        return dragAcceleration.plus(magnusAcceleration).plus(gravityAcceleration);
    }

    private BallState calculateRK4Step(Vector position, Vector velocity) {
        Vector m1, m2, m3, m4, k1, k2, k3, k4;
        m1 = velocity;
        k1 = calculateAcceleration(velocity);
        m2 = velocity.plus(k1.times(dt/2));
        k2 = calculateAcceleration(velocity.plus(k1.times(dt/2)));
        m3 = velocity.plus(k2.times(dt/2));
        k3 = calculateAcceleration(velocity.plus(k2.times(dt/2)));
        m4 = velocity.plus(k3.times(dt));
        k4 = calculateAcceleration(velocity.plus(k3.times(dt)));

        Vector newPosition = m1.plus(m2.times(2)).plus(m3.times(2)).plus(m4).times(dt/6);
        Vector newVelocity = k1.plus(k2.times(2)).plus(k3.times(2)).plus(k4).times(dt/6);

        return new BallState(newPosition, newVelocity);
    }

    public void calculateError() {
        Vector position = VecBuilder.fill(shooterPose.getX(), shooterPose.getY(), Constants.shooterHeight);

        Vector velocity = ballInitialLinearVelocity;
        Vector angularVelocity = ballInitialAngularVelocity;

        boolean hasGoneAboveHub = false;

        while (true) {
            double zPosition = position.get(2);
            boolean isAboveHub = zPosition > Constants.hubZ;

            if (zPosition > Constants.hubZ) {
                hasGoneAboveHub = true;
            }

            if (hasGoneAboveHub && !isAboveHub) {
                break;
            } else if (zPosition < 0) {
                xError = Double.NaN;
                yError = Double.NaN;
                return;
            }

            ballState = calculateRK4Step(position, angularVelocity);

            position = ballState.getPosition();
            velocity = ballState.getVelocity();
        }

        xError = targetPose.getX() - position.get(0);
        yError = targetPose.getY() - position.get(1);
    }

    public double getXError() {
        return xError;
    }

    public double getYError() {
        return yError;
    }
}