package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

/**
 * Robot-wide constants.
 * All distances in meters, all angles in radians, all speeds in m/s.
 */
public final class Constants {

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
    }

    // Drive Constants
    public static final double MaxDrivingSpeed =
        1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularDrivingSpeed =
        1 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** Half-diagonal of robot, used for angular velocity estimation (meters) */
    public static final double robotRadius = 12.5 * 0.0254; // 12.5 inches → meters

    public static final int hoodMotorId = 13;
    public static final int kickerMotorId = 14;
    public static final int levitatorMotorId = 15;
    public static final int feederMotorId = 16;
    public static final int frontShooter1Id = 17;
    public static final int frontShooter2Id = 18;
    public static final int frontShooter3Id = 19;
    public static final int pivotMotorId = 20;
    public static final int rollerMotorId = 21;
    public static final int rollerMotor2Id = 22;

    public static final double intakePivotKP = 0.2;
    public static final double intakePivotKI = 0;
    public static final double intakePivotKD = 0;
    public static final double intakeHardStopAngle = 0;
    public static final double levitatorKP = 0;
    public static final double levitatorKI = 0;
    public static final double levitatorKD = 0;
    public static double shooterMotorKP = 0;
    public static double shooterMotorKI = 0;
    public static double shooterMotorKD = 0;

    public static double hoodPivotKP = 22;
    public static double hoodPivotKI = 4;
    public static double hoodPivotKD = 1;
    public static final double hoodPivotKS = 0.25;
    public static final double hoodPivotKV = 0.0;
    public static double hoodPivotKG = 0.62;
    public static final double hoodPivotKA = 0.0;
    public static final double hoodHardStopAngle = 0.15918;

    public static final double hoodMaxVelocity = 5;
    public static final double hoodMaxAcceleration = 5;

    public static final double FIELD_WIDTH = 8.21;
    public static final double FIELD_LENGTH = 16.54;
    public static final Pose2d blueHubPose = new Pose2d(
        4.61,
        FIELD_WIDTH / 2.0,
        new Rotation2d(0)
    );
    public static final Pose2d redHubPose = new Pose2d(
        16.51 - 4.61,
        FIELD_WIDTH / 2.0,
        new Rotation2d(0)
    );

    // ──Shooter Velocity─────────────────────────────────────────────────────────

    public static final double shootingSpeed = 45.0; //43
    public static final double feedingSpeed = 90.0;
    public static final double shootingTolerance = 1.0;

    // ── Shooting geometry ─────────────────────────────────────────────────────
    /** Height of the hub opening (top of the ball's entry path), meters */
    public static final double hubZ = 1.8288;

    /**
     * Height of the shooter exit point above the field floor, meters. Measure on
     * robot.
     */
    public static final double shooterHeight = 0.3;

    /** Shooter exit position offset from robot center, meters. Measure on robot. */
    public static final double shooterPositionX = 0.2; // forward
    public static final double shooterPositionY = 0.0; // lateral

    /** Ball exit speed from shooter, m/s. Tune from shooter characterization. */
    public static final double ballInitialVelocityFromShooterHub = 9.5;
    public static final double ballInitialVelocityFromShooterNeutralZone = 9.5;
    public static final double ballInitialVelocityFromShooterEnemyZone = 9.5;
    public static final double ballInitialSpinFromShooterHub = 100;
    public static final double ballInitialSpinFromShooterNeutralZone = 100;
    public static final double ballInitialSpinFromShooterEnemyZone = 100;

    // ── Intake positions (encoder units — tune before deploy) ────────────────
    public static final double IntakeDeployPos = 25;
    public static final double IntakeDepotPos = 22;
    public static final double IntakeRetractPos = 10;
    public static final double IntakeStowedPos = 0;

    // Hood positions
    public static final double shootingHoodAngle = 0.088;
    public static final double feedingHoodAngle = 0.08;


    // ── Vision ───────────────────────────────────────────────────────────────
    /**
     * Transform from robot center to camera lens.
     * x = forward, y = left, z = up (meters). Rotation is camera pitch/roll/yaw.
     * Measure actual mount position on robot.
     */
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
        4,
        4,
        Double.POSITIVE_INFINITY
    );
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
        0.5,
        0.5,
        Double.POSITIVE_INFINITY
    );

    public static final Transform3d kRobotToCam = new Transform3d(
        -0.309,
        -0.064,
        0.193,
        new Rotation3d(0, -Math.toRadians(27), Math.PI)
    );

    public static double closestFiducialIgnoreThreshold = 10;

    // ── AprilTag layout ───────────────────────────────────────────────────────
    public static AprilTagFieldLayout kTagLayout;

    static {
        try {} catch (Exception e) {
            e.printStackTrace();
            kTagLayout = null;
        }
    }

    // ── Poses ───────────────────────────────────────────────────
    private static final double x = 2.362, y = 5.205;
    public static final Pose2d targetPoseHubLeft = new Pose2d(
        x,
        y,
        new Rotation2d(
            Constants.blueHubPose.getX() - x,
            Constants.blueHubPose.getY() - y
        ).plus(new Rotation2d(Math.PI))
    );
    public static final Pose2d targetPoseHubRight = new Pose2d(
        x,
        (8 - y),
        new Rotation2d(
            Constants.blueHubPose.getX() - x,
            Constants.blueHubPose.getY() - (8 - y)
        ).plus(new Rotation2d((2 * Math.PI) / 2))
    );
    public static final Pose2d targetPoseTrenchLeft = new Pose2d(
        6,
        7.4,
        new Rotation2d(0)
    );
    public static final Pose2d targetPoseTrenchRight = new Pose2d(
        6,
        0.6,
        new Rotation2d(0)
    );
}
