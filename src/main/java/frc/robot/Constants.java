package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Robot-wide constants.
 * All distances in meters, all angles in radians, all speeds in m/s.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    
    /** Half-diagonal of robot, used for angular velocity estimation (meters) */
    public static final double robotRadius = 12.5 * 0.0254; // 12.5 inches → meters

    public static final int hoodMotorId     = 13;
    public static final int kickerMotorId   = 14;
    public static final int levitatorMotorId = 15;
    public static final int feederMotorId   = 16;
    public static final int frontShooter1Id = 17;
    public static final int frontShooter2Id = 18;
    public static final int backShooterId   = 19;
    public static final int pivotMotorId    = 20;
    public static final int rollerMotorId   = 21;
    
    public static final double intakePivotKP = 0.2;
    public static final double intakePivotKI = 0;
    public static final double intakePivotKD = 0;
    public static final double levitatorKP = 0;
    public static final double levitatorKI = 0;
    public static final double levitatorKD = 0;
    public static final double shooterPivotKP = 0;
    public static final double shooterPivotKI = 0;
    public static final double shooterPivotKD = 0;
    
    public static final double hoodPivotKP = 0.2;
    public static final double hoodPivotKI = 0;
    public static final double hoodPivotKD = 0;
    public static final double hoodPivotKS = 0;
    public static final double hoodPivotKV = 0.0;
    public static final double hoodPivotKG = -0.69;
    public static final double hoodPivotKA = 0.0;
    public static final double hoodHardStopAngle = 1.725;
   
    public static final double hoodMaxVelocity = 5;
    public static final double hoodMaxAcceleration = 5;

    public static final double FIELD_WIDTH = 8.04; // meters
    public static final Pose2d blueHubPose = new Pose2d(4.61, FIELD_WIDTH / 2.0, new Rotation2d(0));
    public static final Pose2d redHubPose  = new Pose2d(16.51 - 4.61, FIELD_WIDTH / 2.0, new Rotation2d(0));

    // ── Shooting geometry ─────────────────────────────────────────────────────
    /** Height of the hub opening (top of the ball's entry path), meters */
    public static final double hubZ = 1.8288;

    /** Height of the shooter exit point above the field floor, meters. Measure on robot. */
    public static final double shooterHeight = 0.3;

    /** Shooter exit position offset from robot center, meters. Measure on robot. */
    public static final double shooterPositionX = 0.2; // forward
    public static final double shooterPositionY = 0.0; // lateral

    /** Ball exit speed from shooter, m/s. Tune from shooter characterization. */
    public static final double ballInitialVelocityFromShooter = 9.5;
    public static final double ballInitialSpinFromShooter = 100;

    // ── Intake positions (encoder units — tune before deploy) ────────────────
    public static final double IntakeDeployPos  = 25;
    public static final double IntakeRetractPos = 0;

    // ── Vision ───────────────────────────────────────────────────────────────
    /**
     * Transform from robot center to camera lens.
     * x = forward, y = left, z = up (meters). Rotation is camera pitch/roll/yaw.
     * Measure actual mount position on robot.
     */
    public static final Transform3d kRobotToCam = new Transform3d(
        0.5, 0.0, 0.5,
        new Rotation3d(0, 0, 0)
    );

    // ── AprilTag layout ───────────────────────────────────────────────────────
    public static AprilTagFieldLayout kTagLayout;
    static {
        try {
          
        } catch (Exception e) {
            e.printStackTrace();
            kTagLayout = null;
        }
    }
}
