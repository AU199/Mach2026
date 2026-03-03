package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FuelSim;
import frc.robot.Sotm.ShotAngles;
import frc.robot.Sotm.IdealTrajectory;
import frc.robot.Sotm.Newton;

public class Shooter extends SubsystemBase{
    TalonFX frontShooter1 = new TalonFX(Constants.frontShooter1Id, "DriveBase");
    TalonFX frontShooter2 = new TalonFX(Constants.frontShooter2Id, "DriveBase");
    TalonFX backShooter = new TalonFX(Constants.backShooterId, "DriveBase");
    
    Hood hoodMotor = new Hood();

    PositionVoltage pivotAngleRequest = new PositionVoltage(0).withSlot(0);
    boolean isBlue;
    Field2d field;
    int tick = 0;
    CommandSwerveDrivetrain drivebase;
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("FuturePose", Pose3d.struct).publish();
    
    Pose2d hubPose;
    double currentYaw = 0;

    public Shooter(CommandSwerveDrivetrain drivetrain, boolean isBlue, Field2d field) {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.shooterPivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.shooterPivotKD;
        this.field = field;
        this.drivebase = drivetrain;
        if (isBlue) {
            hubPose = Constants.blueHubPose;
        }
        else {
            hubPose = Constants.redHubPose;
        }
    };

    public Pose2d getHubPose() {
        return hubPose;
    }

    public Command shooterOn(double speed) {
        return startEnd(() -> {
            frontShooter1.set(speed);
            frontShooter2.set(speed);
            backShooter.set(-speed);
        }, () -> {
            frontShooter1.set(0);
            frontShooter2.set(0);
            backShooter.set(0);
        });
    }

    // public Command pivotMotorOn(double speed) {
    //     return startEnd(() -> {
    //         hoodMotor.set(speed);
    //         System.out.println("running el hood:" + speed);
    //     }, () -> {
    //         hoodMotor.set(-0.025);
    //         System.out.println("par'e correr el hood");
    //     });
    // }
    // public Command setPivotAngle(double angle) {
    //     return startEnd(() -> {
    //         hoodMotor.setControl(pivotAngleRequest.withPosition(angle));
    //     }, () -> {
    //         hoodMotor.set(0);
    //     });
    // }

    private double getTimeFromDist(double dist) {
        return 2/dist;   
    }

    private double getAngleFromDist(double dist) {
        return 200/(dist);
    }

    private double getDistFromHub(Pose2d pose) {
        Transform2d transform = pose.minus(hubPose);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    private Pose2d getFuturePose(Pose2d robotPose, ChassisSpeeds velocity, double airtime) {
        double futureX = robotPose.getX() + velocity.vxMetersPerSecond * airtime;
        double futureY = robotPose.getY() + velocity.vyMetersPerSecond * airtime;
        return new Pose2d(futureX, futureY, new Rotation2d(0));
    }

    private double getYaw(Pose2d futurePose) {
        Transform2d transform = futurePose.minus(hubPose);
        return Math.atan(transform.getY()/transform.getX());
    }
    
    private int iterationCount = 3;

    public Command droneStrike() {
        return run(() -> {
            Pose2d currentPose = drivebase.getState().Pose;
            double hubDist = getDistFromHub(currentPose);
            Pose2d futurePose = new Pose2d();
            double airTime;
            double shootAngle;

            
            ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivebase.getState().Speeds, currentPose.getRotation());

            for (int i = 0; i <= iterationCount; i++) {
               airTime = getTimeFromDist(hubDist);
               futurePose = getFuturePose(currentPose, velocity, airTime);
               hubDist = getDistFromHub(futurePose);
            }
            currentYaw = getYaw(futurePose);
            publisher.set(new Pose3d(futurePose.getX(), futurePose.getY(), 0,new Rotation3d(0,0,currentYaw)));
            shootAngle = getAngleFromDist(hubDist);
            hoodMotor.setHoodPosition(shootAngle);
            // Make it turn
            SmartDashboard.putNumber("shooterangle", shootAngle);
            double shotSpeed = 4;
            FuelSim.getInstance().spawnFuel(new Translation3d(currentPose.getX(),currentPose.getY(),0), new Translation3d(-shotSpeed*Math.cos(currentYaw)*Math.cos(shootAngle),-shotSpeed*Math.sin(currentYaw)*Math.cos(shootAngle),shotSpeed*Math.sin(shootAngle)*2));
        });
    }

    // spinDirection: 1.0 = backspin, -1.0 = topspin
    public Command droneStrikeRK4(Pose2d targetPose, double spinDirection) {
        return run(() -> {
            Pose2d robotPose = drivebase.getState().Pose;
            ChassisSpeeds robotRobotRelativeVelocity = drivebase.getState().Speeds;
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
            if(tick >= 20){
                FuelSim.getInstance().spawnFuel(
                    new Translation3d(robotPose.getX(), robotPose.getY(), Constants.shooterHeight),
                    new Translation3d(ballLinearVelocity)
                );
                tick = 0;
            }
        });
    }

    @Override
    public void periodic(){
        tick += 1;
    }
}