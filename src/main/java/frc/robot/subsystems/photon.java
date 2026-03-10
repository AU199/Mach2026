package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class photon extends SubsystemBase {

    PhotonCamera camera;
    // private final mEstiamateConsumer estConsumer;
    PhotonPoseEstimator photonEstimator;
    private PhotonCameraSim photonCameraSim;
    private VisionSystemSim visionSystemSim;
    private Matrix<N3, N1> curStdevs;
    CommandSwerveDrivetrain swerveDriveBase;
    StructPublisher<Pose3d> cameraPose = NetworkTableInstance.getDefault()
            .getStructTopic("CameraPose Estimate", Pose3d.struct).publish();

    int tick = 0;
    private AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public photon(CommandSwerveDrivetrain swerveDriveBase) {
        this.swerveDriveBase = swerveDriveBase;
        this.camera = new PhotonCamera("MainCamera");
        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.kRobotToCam);

        if (Robot.isSimulation()) {
            visionSystemSim = new VisionSystemSim("main");
            System.out.println(kTagLayout.toString());
            visionSystemSim.addAprilTags(kTagLayout);

            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            photonCameraSim = new PhotonCameraSim(camera, cameraProp);

            visionSystemSim.addCamera(photonCameraSim, Constants.kRobotToCam);
            photonCameraSim.enableDrawWireframe(true);
        }
    }

    /** Processes a single pipeline result and feeds the pose estimate to the drivetrain. */
    private void processResult(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);

        if (visionEstimate.isEmpty()) {
            visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
        }

        visionEstimate.ifPresent(est -> {
            swerveDriveBase.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds);
            cameraPose.accept(est.estimatedPose);
            
        });
    }

    /** Used on real hardware — reads the latest unread results from the camera. */
    private void getResults() {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            processResult(result);
        }
    }

    public void getResultsSim() {
        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
        // System.out.println("Got a result with " + result.getTargets().size() + " targets");
            var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEstimate.isEmpty()) {
                visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            visionEstimate.ifPresentOrElse(
                        est ->
                                {swerveDriveBase.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    cameraPose.accept(est.estimatedPose);
                                }
                        ,
                            System.out::println
                        );
                        
            }


    }

    public Pose2d transform3dToPose2d(Transform3d transforming) {
        return new Pose2d(transforming.getMeasureX(), transforming.getMeasureY(),
                transforming.getRotation().toRotation2d());
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSystemSim.getDebugField();
    }

    @Override
    public void simulationPeriodic() {
        Pose2d robotpose = swerveDriveBase.getState().Pose;
        getResultsSim();
    }
    @Override 
    public void periodic(){
        if (Robot.isSimulation()) {
            getResultsSim();
        } else {
            getResults();
        }
        
    }

}