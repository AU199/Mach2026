package frc.robot.Sotm;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Sotm.ShotAngles;

public class Trajectory {
    double dragCoefficient = 0.47;
    double carlsenCoefficient;
    double ballInitialVelocity = 10;

    Pose2d hubPose;

    double angularVelocity;
    double epsilon = 0.001;

    Pose2d landingPose;
    Pose2d robotPose;
    ChassisSpeeds robotVelocity;

    public Trajectory(Pose2d robotPose, ChassisSpeeds robotVelocity, double angularVelocity) {
        this.robotPose = robotPose;
        this.robotVelocity = robotVelocity;
        this.angularVelocity = angularVelocity;
    }

    public Pose2d getError() {
        return new Pose2d(landingPose.getX() - hubPose.getX(), landingPose.getY() - hubPose.getY(), new Rotation2d(0));
    }

    public ShotAngles getIdealShotAngles() {
        double[] angles = {Math.atan((Math.pow(ballInitialVelocity, 2) + Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(hubPose.getX(), 2) + 2 * hubPose.getY() * Math.pow(ballInitialVelocity, 2)))) / (9.81* hubPose.getX())), Math.atan((Math.pow(ballInitialVelocity, 2) - Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(hubPose.getX(), 2) + 2 * hubPose.getY() * Math.pow(ballInitialVelocity, 2)))) / (9.81* hubPose.getX()))};
        double theta;
        if (angles[0] < 0 || angles[0] > 90) {
            theta = angles[0];
        }
        else {
            theta = angles[1];
        }
        double phi = Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());

        return new ShotAngles(theta, phi);
    }
}