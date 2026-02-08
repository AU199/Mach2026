package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.time.Period;
import java.util.concurrent.Flow.Publisher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.FlatBufferBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FuelSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Shooter extends SubsystemBase {
    TalonFX frontShooter1 = new TalonFX(Constants.frontShooter1Id);
    TalonFX frontShooter2 = new TalonFX(Constants.frontShooter2Id);
    TalonFX backShooter = new TalonFX(Constants.backShooterId);
    TalonFX hoodMotor = new TalonFX(Constants.hoodMotorId);
    PositionVoltage pivotAngleRequest = new PositionVoltage(0).withSlot(0);
    boolean isBlue;
    Field2d field;
    double shooterVelocity = 10;
    CommandSwerveDrivetrain drivebase;
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("FuturePose", Pose3d.struct).publish();
    StructPublisher<Pose3d> hubPositionPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Hub Position", Pose3d.struct).publish();
    Pose2d hubPose;
    double currentYaw = 0;
    Simulation simulation;
    int tick;
    private Float epsilon = 0.0001f;
    // private static final InterpolatingTreeMap LOOKUP_TABLE = new
    // InterpolatingTreeMap(null, null);

    public Shooter(CommandSwerveDrivetrain drivetrain, boolean isBlue, Field2d field) {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.pivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.pivotKD;
        this.field = field;
        this.drivebase = drivetrain;
        this.isBlue = isBlue;
        if (isBlue) {
            hubPose = new Pose2d(182.11 * 0.0254, 158.84 * 0.0254, new Rotation2d(0));
        } else {
            hubPose = new Pose2d(469.11 * 0.0254, 158.84 * 0.0254, new Rotation2d(0));
        }
        this.simulation = new Simulation(0.46, 0.2, new Translation3d(0, 45, 0),
                new Pose3d(hubPose.getX(), hubPose.getY(), (72) / 39.37, new Rotation3d(0, 0, 0)));

    }

    public Command shooterOn() {
        return startEnd(() -> {
            frontShooter1.set(1);
            frontShooter2.set(1);
            backShooter.set(1);
        }, () -> {
            frontShooter1.set(0);
            frontShooter2.set(0);
            backShooter.set(0);
        });
    }

    public Command setPivotAngle(double angle) {
        return startEnd(() -> {
            hoodMotor.setControl(pivotAngleRequest.withPosition(angle));
        }, () -> {
            hoodMotor.set(0);
        });
    }

    private double getTimeFromDist(double dist, double velocity) {
        return dist / velocity;
    }

    private double getAngleFromDist(double dist, double velocity) {
        return Math.max(Math.toRadians(35),Math.min(Math.atan((Math.pow(velocity, 2)
                + Math.sqrt(Math.pow(velocity, 4) - Constants.G * (Constants.G * Math.pow(dist, 2)
                        + 2 * Math.pow(velocity, 2) * (Constants.HeightFromShooterToHubINCHES / 39.37))))
                / (Constants.G * dist)),Math.toRadians(75)));
    }

    private double getDistFromHub(Pose2d pose) {
        Transform2d transform = pose.minus(hubPose);
        return Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    private Pose2d getFuturePose(Pose2d pose, ChassisSpeeds velocity, double airtime) {
        double futureX = pose.getX() + velocity.vxMetersPerSecond * airtime;
        double futureY = pose.getY() + velocity.vyMetersPerSecond * airtime;
        return new Pose2d(futureX, futureY, new Rotation2d(0));
    }

    private double getYaw(Pose2d futurePose) {
        Transform2d transform = futurePose.minus(hubPose);
        return Math.atan(transform.getY()/transform.getX());
    }

    private void spawnFuel(Pose2d currentPose, double shootAngle) {
        FuelSim.getInstance().spawnFuel(new Translation3d(currentPose.getX(), currentPose.getY(), 0),
                new Translation3d(shooterVelocity * Math.cos(currentYaw) * Math.cos(shootAngle),
                        shooterVelocity * Math.sin(currentYaw) * Math.cos(shootAngle),
                        Math.sin(shootAngle) * shooterVelocity),
                new Translation3d(0, 45, 0));

    }

    private int iterationCount = 3;

    public Command droneStrike() {
        return run(() -> {
            Pose2d currentPose = drivebase.getState().Pose;
            double hubDist = getDistFromHub(currentPose);
            Pose2d futurePose = new Pose2d();
            double airTime;
            double shootAngle;

            ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(drivebase.getState().Speeds,
                    currentPose.getRotation());

            for (int i = 0; i <= iterationCount; i++) {
                airTime = getTimeFromDist(hubDist, shooterVelocity);
                futurePose = getFuturePose(currentPose, velocity, airTime);
                hubDist = getDistFromHub(futurePose);
            }
            currentYaw = getYaw(futurePose);
            shootAngle = getAngleFromDist(hubDist, shooterVelocity);
            SmartDashboard.putNumber("shooterangleBefore", Math.toDegrees(shootAngle));
            SmartDashboard.putNumber("hub height changer", hubDist/10);
            publisher.set(new Pose3d(futurePose.getX(), futurePose.getY(), 0, new Rotation3d(0, 0, currentYaw)));

            if (!Double.isNaN(shootAngle)){
                double[] shooterAnglePhi = simulation.findThetaPhi(shooterVelocity,
                    new Pose3d(futurePose.getX(), futurePose.getY(), 0, new Rotation3d(0, 0, currentYaw)), currentYaw,0,shootAngle,
                    epsilon, 10, 0.01,0.5);
                    
                shootAngle = shooterAnglePhi[0];
                currentYaw = shooterAnglePhi[1];

                hoodMotor.setControl(pivotAngleRequest.withPosition(shootAngle));
                // shootAngle = 1-shootAngle;
                SmartDashboard.putNumber("shooterangle", Math.toDegrees(shootAngle));
                if (tick >= 20 && ((Math.toRadians(35) <= shootAngle) && (shootAngle<= Math.toDegrees(75)))) {
                    spawnFuel(currentPose, shootAngle);
                    tick = 0;
                }
            }else{
                System.out.println("VELOCITY NOT ENOUGH");
            }
        });
    }

    @Override
    public void periodic() {
        tick += 1;
        SmartDashboard.putNumber("tick", tick);
        FuelSim.getInstance().stepSim();
        hubPositionPublisher.accept(new Pose3d(hubPose.getX(), hubPose.getY(), 72 / 39.37, new Rotation3d(0, 0, 0)));

    }

    public void IncreaseSpeed() {
        if (tick == 100){
            this.shooterVelocity += 1;
            }    
        }

    public void DecreaseSpeed() {
        if (tick == 100){
            this.shooterVelocity -= 1;
        }
    }
}