package frc.robot.Sotm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Projects robot pose forward in time to compensate for camera/processing latency.
 */
public class LatencyCompensation {

    public LatencyCompensation() {}

    /**
     * Returns an estimated current pose given a pose observed at some time in the past.
     *
     * @param observedPose   The pose as measured (potentially stale)
     * @param fieldSpeeds    Current field-relative chassis speeds
     * @param latencySeconds How many seconds ago the pose was observed
     * @return Estimated current pose
     */
    public Pose2d compensate(Pose2d observedPose, ChassisSpeeds fieldSpeeds, double latencySeconds) {
        double dx = fieldSpeeds.vxMetersPerSecond * latencySeconds;
        double dy = fieldSpeeds.vyMetersPerSecond * latencySeconds;
        double dtheta = fieldSpeeds.omegaRadiansPerSecond * latencySeconds;

        return new Pose2d(
            observedPose.getX() + dx,
            observedPose.getY() + dy,
            observedPose.getRotation().plus(new Rotation2d(dtheta))
        );
    }
}
