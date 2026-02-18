package frc.robot.Sotm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Sotm.RK4;
import frc.robot.Sotm.ShotAngles;
import frc.robot.Sotm.Trajectory;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Newton {
    private ChassisSpeeds robotRobotRelativeVelocity;
    private ChassisSpeeds robotFieldRelativeVelocity;
    
    private Vector ballInitialLinearVelocityRelativeToRobot;
    private Vector ballInitialAngularVelocityRelativeToRobot;
    private Vector ballInitialLinearVelocityRelativeToField;
    private Vector ballInitialAngularVelocityRelativeToField;

    private Pose2d shooterPose;
    private Pose2d robotPose;
    private Pose2d hubPose;
    private Pose2d targetPose;

    private double dt = 0.05;
    private double epsilon = 0.001;

    public Newton(Pose2d shooterPose, Pose2d robotPose, Pose2d hubPose, ChassisSpeeds robotVelocity) {
        this.shooterPose = shooterPose;
        this.robotPose = robotPose;
        this.robotRobotRelativeVelocity = robotVelocity;
        this.robotFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, robotPose.getRotation());
        this.hubPose = hubPose;
    }

    public BallError calculateError(double theta, double phi, double ballInitialLinearSpeedRelativeToShooter, double ballInitialAngularSpeedRelativeToShooter) {
        ballInitialLinearVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.cos(phi), 
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.sin(theta),  // Maybe sin(phi)
            ballInitialLinearSpeedRelativeToShooter * Math.sin(theta));

        ballInitialAngularVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialAngularSpeedRelativeToShooter * Math.cos(phi),
            ballInitialAngularSpeedRelativeToShooter * Math.sin(theta));

        ballInitialLinearVelocityRelativeToField = VecBuilder.fill(
            robotFieldRelativeVelocity.vxMetersPerSecond,
            robotFieldRelativeVelocity.vyMetersPerSecond, 0)
            .plus(ballInitialLinearVelocityRelativeToRobot);

        ballInitialAngularVelocityRelativeToField = VecBuilder.fill(1); // Find this later

        targetPose = hubPose; // Add ability to change this later
        RK4 rk4 = new RK4(targetPose, robotFieldRelativeVelocity, ballInitialLinearVelocityRelativeToField, ballInitialAngularVelocityRelativeToField, shooterPose, dt);

        return rk4.calculateError();
    }

    public ShotAngles findOptimalTrajectory() {
        double angularVelocity = robotFieldRelativeVelocity.omegaRadiansPerSecond;
        // double angularVelocity = Math.sqrt(Math.pow(robotFieldRelativeVelocity.vxMetersPerSecond, 2) + Math.pow(robotFieldRelativeVelocity.vyMetersPerSecond, 2))/Constants.robotRadius;
        Trajectory trajectory = new Trajectory(robotPose, robotFieldRelativeVelocity, angularVelocity);
        ShotAngles idealShotAngle = trajectory.getIdealShotAngles();
        BallError idealErrors, thetaAdjustedErrors, phiAdjustedErrors;

        double ballInitialVelocity = Constants.ballInitialVelocity; // Find launch speed later
        idealErrors = calculateError(idealShotAngle.getTheta(), idealShotAngle.getPhi(), ballInitialVelocity, angularVelocity);
        thetaAdjustedErrors = calculateError(idealShotAngle.getTheta() + epsilon, idealShotAngle.getPhi(), ballInitialVelocity, angularVelocity);
        phiAdjustedErrors = calculateError(idealShotAngle.getTheta(), idealShotAngle.getPhi() + epsilon, ballInitialVelocity, angularVelocity);

        Derivative thetaDerivatives = new Derivative(thetaAdjustedErrors.getxError() - idealErrors.getxError(), thetaAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);
        Derivative phiDerivatives = new Derivative(phiAdjustedErrors.getxError() - idealErrors.getxError(), phiAdjustedErrors.getyError() - idealErrors.getyError(), epsilon);

        // Jacobians might be wrong
        double optimalPhi = (thetaDerivatives.getDerivatives()[1] * (idealErrors.getxError() - hubPose.getX()) - thetaDerivatives.getDerivatives()[0] - (idealErrors.getyError() - hubPose.getY())) / (thetaDerivatives.getDerivatives()[0] * phiDerivatives.getDerivatives()[1] - thetaDerivatives.getDerivatives()[1] * phiDerivatives.getDerivatives()[0]);
        double optimalTheta = (-(idealErrors.getyError() - hubPose.getY()) - optimalPhi * phiDerivatives.getDerivatives()[1]) / thetaDerivatives.getDerivatives()[1];

        return new ShotAngles(optimalTheta, optimalPhi);
    }
}