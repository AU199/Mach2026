package frc.robot.Sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import edu.wpi.first.math.Vector;

public class IdealTrajectory {
    double dragCoefficient = 0.47;
    double carlsenCoefficient;
    double ballInitialVelocity = 10;

    Pose2d hubPose;

    double epsilon = 0.001;

    Pose2d landingPose;
    Pose2d robotPose;
    ChassisSpeeds robotVelocity;

    public IdealTrajectory(Pose2d robotPose, Pose2d hubPose, ChassisSpeeds robotVelocity) {
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

        // double[] angles = {Math.atan((Math.pow(ballInitialVelocity, 2) + Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(horizontalDistance, 2) + 2 * heightDifference * Math.pow(ballInitialVelocity, 2)))) / (9.81* horizontalDistance)), Math.atan((Math.pow(ballInitialVelocity, 2) - Math.sqrt(Math.pow(ballInitialVelocity, 4) - 9.81* (9.81* Math.pow(horizontalDistance, 2) + 2 * heightDifference * Math.pow(ballInitialVelocity, 2)))) / (9.81* horizontalDistance))};
        double discriminant = Math.pow(ballInitialVelocity, 4) - 9.81 * (9.81 * Math.pow(horizontalDistance, 2) + 2 * heightDifference * Math.pow(ballInitialVelocity, 2));
        if (discriminant < 0) {
            return new ShotAngles(Double.NaN, Double.NaN);
        }
        double sqrtDisc = Math.sqrt(discriminant);
        double[] angles = {
            Math.atan((Math.pow(ballInitialVelocity, 2) + sqrtDisc) / (9.81 * horizontalDistance)),
            Math.atan((Math.pow(ballInitialVelocity, 2) - sqrtDisc) / (9.81 * horizontalDistance))};
        
        double theta = angles[0];
        // if (angles[1] < 0 || angles[1] > Math.PI/2) {
        //     theta = angles[0]; // High angle
        // } else {
        //     theta = angles[1];  // Low angle
        // }

        // double phi = Math.atan2(heightDifference - robotPose.getY(), horizontalDistance - robotPose.getX());
        double flightTime = horizontalDistance / (ballInitialVelocity * Math.cos(theta));
        double adjustedDx = dx - robotVelocity.vxMetersPerSecond * flightTime;
        double adjustedDy = dy - robotVelocity.vyMetersPerSecond * flightTime;
        double phi = Math.atan2(adjustedDy, adjustedDx);

        return new ShotAngles(theta, phi);
    }
}