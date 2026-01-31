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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class photon extends SubsystemBase{
        
    PhotonCamera camera;
    PhotonPoseEstimator photonEstimator;
    private Matrix<N3,N1> curStdevs;
    CommandSwerveDrivetrain swerveDriveBase;


    public photon(CommandSwerveDrivetrain swerveDriveBase ){
        this.swerveDriveBase = swerveDriveBase;
        this.camera = new PhotonCamera("photonvision");
        photonEstimator =
                new PhotonPoseEstimator(Constants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    public void getResults(){
        var visionEstimates = camera.getAllUnreadResults();
        for (var visionEstimate: visionEstimates){    
                var multiTagVision = visionEstimate.multitagResult;
                if(multiTagVision.isPresent()){
                    var transform3dPos = multiTagVision.get().estimatedPose.best;    
                    Pose2d pose2dPos = transform3dToPose2d(transform3dPos);
                    System.out.println("Estimated x: " +pose2dPos.getX() + "\nEstimated y: "+pose2dPos.getY() + "\nEstimated yaw: "+ pose2dPos.getRotation());
                // swerveDrive.addVisionMeasurement(pose2dPos, 0);
                }
    }

    }

    public Pose2d transform3dToPose2d(Transform3d transforming){
        return new Pose2d(transforming.getMeasureX(),transforming.getMeasureY(),transforming.getRotation().toRotation2d());
    }
    
    
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return null;
    }
        

    @Override
    public void periodic(){
        getResults();
    }

}
