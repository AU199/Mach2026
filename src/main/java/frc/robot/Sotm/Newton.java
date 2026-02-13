package frc.robot.Sotm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Sotm.RK4;
import frc.robot.Sotm.ShotAngles;
import frc.robot.Sotm.Trajectory;

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

    public void calculateError(double theta, double phi, double ballInitialLinearSpeedRelativeToShooter, double ballInitialAngularSpeedRelativeToShooter) {
        ballInitialLinearVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.cos(phi), 
            ballInitialLinearSpeedRelativeToShooter * Math.cos(theta) * Math.sin(theta), 
            ballInitialLinearSpeedRelativeToShooter * Math.sin(theta));

        ballInitialAngularVelocityRelativeToRobot = VecBuilder.fill(
            ballInitialAngularSpeedRelativeToShooter * Math.cos(phi),
            ballInitialAngularSpeedRelativeToShooter * Math.sin(theta));

        ballInitialLinearVelocityRelativeToField = VecBuilder.fill(
            robotFieldRelativeVelocity.vxMetersPerSecond,
            robotFieldRelativeVelocity.vyMetersPerSecond, 0)
            .plus(ballInitialLinearVelocityRelativeToRobot);

        targetPose = hubPose; // Probably changing this later
        RK4 rk4 = new RK4(targetPose, robotFieldRelativeVelocity, ballInitialLinearVelocityRelativeToField, ballInitialAngularVelocityRelativeToField, shooterPose, dt);

        rk4.calculateError();
    }

    public ShotAngles findOptimalTrajectory() {
        Trajectory trajectory = new Trajectory(robotPose, robotFieldRelativeVelocity, Math.sqrt(Math.pow(robotFieldRelativeVelocity.vxMetersPerSecond, 2) + Math.pow(robotFieldRelativeVelocity.vyMetersPerSecond, 2))/Constants.robotRadius);
        ShotAngles idealShotAngle = trajectory.getIdealShotAngles();
    }
}