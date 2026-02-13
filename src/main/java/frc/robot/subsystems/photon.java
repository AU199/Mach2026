package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class photon extends SubsystemBase{
        
    PhotonCamera camera;
    // private final mEstiamateConsumer estConsumer;
    PhotonPoseEstimator photonEstimator;
    private Matrix<N3,N1> curStdevs;
    CommandSwerveDrivetrain swerveDriveBase;
    StructPublisher<Pose3d> cameraPose =  NetworkTableInstance.getDefault()
            .getStructTopic("CameraPose Estimate", Pose3d.struct).publish();

    int tick = 0;
    public photon(CommandSwerveDrivetrain swerveDriveBase){
        this.swerveDriveBase = swerveDriveBase;
        this.camera = new PhotonCamera("MainCamera");
        photonEstimator =
                new PhotonPoseEstimator(Constants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    public void getResults(){
        var VisionEstimates = camera.getAllUnreadResults();
        for(var VisionEstimate:VisionEstimates){
            Optional<EstimatedRobotPose> estimatedPoseInitial = photonEstimator.estimateCoprocMultiTagPose(VisionEstimate);   
            Matrix<N3,N3> cameMatrix = camera.getCameraMatrix().orElse(null);
            if (cameMatrix == null){
                System.out.println("NO CAM MATRIX");
            }
            Matrix<N8,N1> distOffsetMatrix = camera.getDistCoeffs().orElse(null);
            if(distOffsetMatrix == null){
                System.out.println("NO DIST OFFSET");
            }
            Optional<EstimatedRobotPose> estimatedPoseBetter = photonEstimator.estimateConstrainedSolvepnpPose(VisionEstimate, cameMatrix, distOffsetMatrix, estimatedPoseInitial.get().estimatedPose, false, 1.0);
        }
        
    }


    public Pose2d transform3dToPose2d(Transform3d transforming){
        return new Pose2d(transforming.getMeasureX(),transforming.getMeasureY(),transforming.getRotation().toRotation2d());
    }
    
    
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return null;
    }
        
    // @FunctionalInterface
    // public static interface mEstiamateConsumer{
    //     public void accept(Pose2d pose, double timestamp, Matrix<N3,N1>estimationSTDevs);
    // }

    @Override
    public void periodic(){
        tick += 1;
        if(tick >=100){
            getResults();
            tick = 0;
    }
    }   

}
