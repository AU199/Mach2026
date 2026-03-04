package frc.robot.Sotm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FuelSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;

public class DroneStrike extends Command{
    public CommandSwerveDrivetrain drivetrain;
    public Pose2d targetPose;
    public Hood hoodMotor;
    public double spinDirection;
        
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault().getStructTopic("FuturePose", Pose3d.struct).publish();

    public DroneStrike(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, Hood hoodMotor, double spinDirection) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.hoodMotor = hoodMotor;
        this.spinDirection = spinDirection;
    }

    @Override
    public void initialize() {}

    // spinDirection: 1.0 = backspin, -1.0 = topspin
    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
            ChassisSpeeds robotRobotRelativeVelocity = drivetrain.getState().Speeds;
            ChassisSpeeds robotFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRobotRelativeVelocity, robotPose.getRotation());

            IdealTrajectory idealTrajectory = new IdealTrajectory(robotPose, targetPose, robotFieldRelativeVelocity);
            ShotAngles currentAngles = idealTrajectory.getIdealShotAngles();

            double theta = currentAngles.getTheta();
            double phi   = currentAngles.getPhi();

            SmartDashboard.putNumber("Ideal Theta", theta);
            SmartDashboard.putNumber("Ideal Phi",  phi);

            Newton newton = new Newton(robotPose, targetPose, robotFieldRelativeVelocity, spinDirection);

            ShotAngles anglesFromNewton = newton.findOptimalTrajectory(currentAngles);
            if (!(Double.isNaN(anglesFromNewton.getTheta()) || Double.isNaN(anglesFromNewton.getPhi()))) {
                System.out.println("Newton returned");
                currentAngles = anglesFromNewton;
            }
            else {
                System.out.println("Shot was NaN");
                return;
            }
            
            theta = currentAngles.getTheta();
            phi = currentAngles.getPhi();
            SmartDashboard.putNumber("Newton Theta", theta);
            SmartDashboard.putNumber("Newton Phi", phi);            

            hoodMotor.setHoodPosition(theta);
            publisher.set(new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, phi)));

            double shotSpeed = Constants.ballInitialVelocityFromShooter;
            
            // Spawn fuel ball in FuelSim with velocity from shot angles

            Vector ballLinearVelocity = newton.getFinalBallLinearVelocity();
            if (ballLinearVelocity == null) {
                System.out.println("Ball Velocity Null");
                return;
            }
            
            FuelSim.getInstance().spawnFuel(
                    new Translation3d(robotPose.getX(), robotPose.getY(), Constants.shooterHeight),
                    new Translation3d(ballLinearVelocity)
            );
    }

    @Override
    public void end(boolean interupted) {}
}
