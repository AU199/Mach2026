package frc.robot.Sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import edu.wpi.first.math.Vector;

public class Trajectory {
    double dragCoefficient = 0.47;
    double carlsenCoefficient;
    double ballInitialVelocity = 10;

    Pose2d hubPose;

    double epsilon = 0.001;

    Pose2d landingPose;
    Pose2d robotPose;
    ChassisSpeeds robotVelocity;

    public Trajectory(Pose2d robotPose, Pose2d hubPose,ChassisSpeeds robotVelocity) {
        this.robotPose = robotPose;
        this.hubPose = hubPose;
        this.robotVelocity = robotVelocity;
    }

    public ShotAngles getIdealShotAngles() {
        Transform2d delta = hubPose.minus(robotPose);

        double dx = delta.getX();
        double dy = delta.getY();
        double horizontalDistance = Math.sqrt(dx*dx + dy*dy);
        double heightDifference = Constants.hubZ - Constants.shooterHeight;

        double[] angles = {Math.atan((Math.pow(ballInitialVelocity, 2) + Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(horizontalDistance, 2) + 2 * heightDifference * Math.pow(ballInitialVelocity, 2)))) / (9.81* horizontalDistance)), Math.atan((Math.pow(ballInitialVelocity, 2) - Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(horizontalDistance, 2) + 2 * heightDifference * Math.pow(ballInitialVelocity, 2)))) / (9.81* horizontalDistance))};
        double theta;
        if (angles[1] < 0 || angles[1] > Math.PI/2) {
            theta = angles[0]; // High angle
        } else {
            theta = angles[1];  // Low angle
        }

        double phi = Math.atan2(dy, dx);

        return new ShotAngles(theta, phi);
    }
}