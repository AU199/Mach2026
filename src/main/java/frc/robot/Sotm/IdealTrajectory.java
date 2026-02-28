
package frc.robot.Sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/*
 * Uses standard vacuum projectile equations:
 *   theta = atan((v² ± sqrt(v^4 - g(g*d² + 2*h*v²))) / (g*d))
 *   phi   = atan2(dy, dx)  — field-relative azimuth to hub
 */
public class IdealTrajectory {

    private final Pose2d robotPose;
    private final Pose2d hubPose;

    // Ball speed used for the analytic guess — same as Newton uses
    private final double ballSpeed = Constants.ballInitialVelocityFromShooter;

    public IdealTrajectory(Pose2d robotPose, Pose2d hubPose, ChassisSpeeds robotVelocity) {
        this.robotPose = robotPose;
        this.hubPose = hubPose;
        // robotVelocity is intentionally not used in the analytic vacuum solution;
        // Newton's method accounts for robot velocity in the full physics simulation.
    }

   
    public ShotAngles getIdealShotAngles() {
        // ── Field-relative displacement to hub ──
        double dx = hubPose.getX() - robotPose.getX();
        double dy = hubPose.getY() - robotPose.getY();
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
        double heightDifference = Constants.hubZ - Constants.shooterHeight;

        // ── Field-relative azimuth ──
        // atan2(dy, dx) gives the angle from the field +X axis — correct field frame
        double phi = Math.atan2(dy, dx);

       
        // theta = atan((v² ± sqrt(v^4 - g*(g*d² + 2*h*v²))) / (g*d))
        double g = 9.81;
        double v2 = ballSpeed * ballSpeed;
        double v4 = v2 * v2;
        double discriminant = v4 - g * (g * horizontalDistance * horizontalDistance
                                        + 2.0 * heightDifference * v2);

        if (discriminant < 0) {
            
            return new ShotAngles(Math.PI / 4.0, phi);
        }
        double sqrtDisc = Math.sqrt(discriminant);
        double denom = g * horizontalDistance;
        double thetaHigh = Math.atan((v2 + sqrtDisc) / denom);
        double thetaLow  = Math.atan((v2 - sqrtDisc) / denom);
        double theta = thetaHigh;

        if (theta > 75) {
            return new ShotAngles(Double.NaN, Double.NaN);
        }
        
        return new ShotAngles(theta, phi);
    }
}
