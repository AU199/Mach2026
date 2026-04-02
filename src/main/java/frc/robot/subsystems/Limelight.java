package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    private CommandSwerveDrivetrain drivetrain;
    StructPublisher<Pose2d> limelightPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("limelightOdometry", Pose2d.struct)
            .publish();

    public Limelight(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;

        SwerveDriveState state = drivetrain.getState();
        double yawDegrees = state.Pose.getRotation().getDegrees();

        SmartDashboard.putNumber("Gyro Rotation", yawDegrees);

        LimelightHelpers.SetRobotOrientation("", yawDegrees, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        if (mt2 == null) {
            // System.out.println("limelight no worky");
            return;
        }

        LimelightHelpers.RawFiducial[] rawFiducials =
            LimelightHelpers.getRawFiducials("");

        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        double closestFiducial = Double.POSITIVE_INFINITY;
        for (RawFiducial fiducial : rawFiducials) {
            if (fiducial.distToCamera < closestFiducial) {
                closestFiducial = fiducial.distToCamera;
            }
        }

        if (closestFiducial > Constants.closestFiducialIgnoreThreshold) {
            doRejectUpdate = true;
        }

        SmartDashboard.putBoolean("doRejectUpdate", doRejectUpdate);

        if (!doRejectUpdate) {
            SmartDashboard.putNumber("closestFiducial", closestFiducial);

            double[] stds = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("stddevs")
                .getDoubleArray(new double[9]);

            // Limelight stddevs array is [tx, ty, tz, rx, ry, rz, ...] — indices 0 and 1 are x/y
            SmartDashboard.putNumber("std x", stds[0]);
            SmartDashboard.putNumber("std y", stds[1]);

            SmartDashboard.putNumber(
                "mt2 rotation2d",
                mt2.pose.getRotation().getDegrees()
            );

            drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(stds[0], stds[1], 9999999)
            );
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);

            limelightPublisher.set(mt2.pose);
        }
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }
}
