package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FuelSim;
import frc.robot.subsystems.CommandSwerveDrivetrain.POSITIONS;

public class Shooter extends SubsystemBase {

    public enum ShooterStates {
        Idle,
        SpinningUp,
        Feeding,
        Shooting,
    }

    TalonFX frontShooter1 = new TalonFX(Constants.frontShooter1Id, "DriveBase");
    TalonFX frontShooter2 = new TalonFX(Constants.frontShooter2Id, "DriveBase");
    TalonFX frontShooter3 = new TalonFX(Constants.frontShooter3Id, "DriveBase");

    boolean isBlue;
    Field2d field;
    int tick = 0;
    CommandSwerveDrivetrain drivebase;
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("FuturePose", Pose3d.struct)
            .publish();

    Pose2d hubPose;
    double currentYaw = 0;
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    private double tolerance = 0.5;
    private ShooterStates shooterState = ShooterStates.Idle;
    private double poseFromTrench = 0;

    public Shooter(
            CommandSwerveDrivetrain drivetrain,
            boolean isBlue,
            Field2d field) {
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 7.0 / 57.0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.5; // Constants.shooterMotorKP; // A position error of 2.5 rotations results in 12
                               // V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        var magicMotionConfigs = talonFXConfigs.MotionMagic;
        magicMotionConfigs.MotionMagicAcceleration = 40;
        magicMotionConfigs.MotionMagicJerk = 30;

        frontShooter1.getConfigurator().apply(talonFXConfigs);
        frontShooter2.getConfigurator().apply(talonFXConfigs);
        frontShooter3.getConfigurator().apply(talonFXConfigs);

        frontShooter2.setControl(
                new Follower(Constants.frontShooter1Id, MotorAlignmentValue.Aligned));
        frontShooter3.setControl(
                new Follower(Constants.frontShooter1Id, MotorAlignmentValue.Aligned));
        SmartDashboard.putNumber("shooter/kp", Constants.shooterMotorKP);
        SmartDashboard.putNumber("shooter/ki", Constants.shooterMotorKI);
        SmartDashboard.putNumber("shooter/kd", Constants.shooterMotorKD);

        this.field = field;
        this.drivebase = drivetrain;
        if (isBlue) {
            hubPose = Constants.blueHubPose;
        } else {
            hubPose = Constants.redHubPose;
        }
    }

    public Pose2d getHubPose() {
        return hubPose;
    }

    // public Command shooterOn(double speed) {
    // return startEnd(() -> {
    // frontShooter1.setVoltage(speed);
    // frontShooter2.setVoltage(speed);
    // frontShooter3.setVoltage(speed);

    // }, () -> {
    // frontShooter1.set(0);
    // frontShooter2.set(0);
    // frontShooter3.set(0);
    // });
    // }

    public Command shooterOn(double speed) {
        return startEnd(
                () -> {
                    frontShooter1.setControl(new MotionMagicVelocityVoltage(speed));
                    // frontShooter2.setControl(new MotionMagicVelocityVoltage(speed));
                    // frontShooter3.setControl(new MotionMagicVelocityVoltage(speed));
                },
                () -> {
                    frontShooter1.set(0);
                    // frontShooter2.set(0);
                    // frontShooter3.set(0);
                });
    }

    public Command shootFuel() {
        return Commands.startRun(
                () -> {
                    shooterState = ShooterStates.SpinningUp;
                    frontShooter1.setControl(
                            new MotionMagicVelocityVoltage(Constants.shootingSpeed));
                    // frontShooter2.setControl(new MotionMagicVelocityVoltage(speed));
                    // frontShooter3.setControl(new MotionMagicVelocityVoltage(speed));
                },
                () -> {
                    double velocityError = frontShooter1.getClosedLoopError().getValueAsDouble();
                    if (velocityError < Constants.shootingTolerance) {
                        shooterState = ShooterStates.Shooting;
                    }
                    SmartDashboard.putNumber("Shooting Velocity Error", velocityError);
                });
    }

    public Command feedFuel() {
        return Commands.startRun(
                () -> {
                    shooterState = ShooterStates.SpinningUp;
                    // frontShooter2.setControl(new MotionMagicVelocityVoltage(speed));
                    // frontShooter3.setControl(new MotionMagicVelocityVoltage(speed));
                },
                () -> {
                    double targetVelocity = 0;
                    frontShooter1.setControl(
                            new MotionMagicVelocityVoltage(targetVelocity));
                    double velocityError = frontShooter1.getClosedLoopError().getValueAsDouble();
                    if (velocityError < Constants.shootingTolerance) {
                        shooterState = ShooterStates.Shooting;
                    }
                    SmartDashboard.putNumber("Feeding Velocity Error", velocityError);
                });
    }

    public Command idle() {
        return Commands.runOnce(
                () -> {
                    frontShooter1.setControl(new CoastOut());
                    shooterState = ShooterStates.Idle;
                });
    }

    // private double calculateFeedingVelocity() {
    // // Fake data go collect this later
    // double[] distances = {5};
    // double[] velocities = {30};

    // int totalDistances = distances.length;
    // double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    // for (int i = 0; i < totalDistances; i++) {
    // sumX += distances[i];
    // sumY += velocities[i];
    // sumXY += distances[i] * velocities[i];
    // sumX2 += distances[i] * distances[i];
    // }

    // double slopeOfRegressionLine = (totalDistances * sumXY - sumX * sumY) /
    // (totalDistances * sumX2 - sumX * sumX);
    // double interceptOfRegressionLine = (sumY - slopeOfRegressionLine * sumX) /
    // totalDistances;

    // double totalShootingDistance = isBlue?
    // Math.abs(drivebase.getState().Pose.getX() - 40) :
    // Math.abs(drivebase.getState().Pose.getX() - 90);
    // double feedingVelocity = slopeOfRegressionLine * totalShootingDistance +
    // interceptOfRegressionLine;

    // return feedingVelocity;
    // }

    private double calculateFeedingVelocity() {
        if((drivebase.isAllianceRed().getAsBoolean() && (drivebase.getPositionState().get() == POSITIONS.RED_BOTTOM || drivebase.getPositionState().get() == POSITIONS.RED_TOP) || (!drivebase.isAllianceRed().getAsBoolean() && (drivebase.getPositionState().get() == POSITIONS.BLUE_BOTTOM || drivebase.getPositionState().get() == POSITIONS.BLUE_TOP)))){
            return 0;
        }
        if ((!drivebase.isAllianceRed().getAsBoolean()
                && (drivebase.getPositionState().get() == POSITIONS.RED_TOP
                        || drivebase.getPositionState().get() == POSITIONS.RED_BOTTOM)
                || (drivebase.isAllianceRed().getAsBoolean()
                        && (drivebase.getPositionState().get() == POSITIONS.BLUE_TOP
                                || drivebase.getPositionState().get() == POSITIONS.BLUE_BOTTOM)))) {
            return 90;
        } else {
            poseFromTrench = Math.abs(
                    drivebase.getState().Pose.getX() - (!drivebase.isAllianceRed().getAsBoolean() ? 12.5 : 4.175));
            double m = 90 / 8.195;
            double speed = -m * poseFromTrench + 90;

            return speed;

        }
    }

    // public Command pivotMotorOn(double speed) {
    // return startEnd(() -> {
    // hoodMotor.set(speed);
    // System.out.println("running el hood:" + speed);
    // }, () -> {
    // hoodMotor.set(-0.025);
    // System.out.println("par'e correr el hood");
    // });
    // }
    // public Command setPivotAngle(double angle) {
    // return startEnd(() -> {
    // hoodMotor.setControl(pivotAngleRequest.withPosition(angle));
    // }, () -> {
    // hoodMotor.set(0);
    // });
    // }

    private double getTimeFromDist(double dist) {
        return 2 / dist;
    }

    private double getAngleFromDist(double dist) {
        return 200 / (dist);
    }

    private double getDistFromHub(Pose2d pose) {
        Transform2d transform = pose.minus(hubPose);
        return Math.sqrt(
                Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));
    }

    public BooleanSupplier atSpeed(double targetSpeed) {
        return () -> {
            return (Math.abs(
                    frontShooter1.getVelocity().getValueAsDouble() - targetSpeed) < tolerance);
        };
    }

    private Pose2d getFuturePose(
            Pose2d robotPose,
            ChassisSpeeds velocity,
            double airtime) {
        double futureX = robotPose.getX() + velocity.vxMetersPerSecond * airtime;
        double futureY = robotPose.getY() + velocity.vyMetersPerSecond * airtime;
        return new Pose2d(futureX, futureY, new Rotation2d(0));
    }

    private double getYaw(Pose2d futurePose) {
        Transform2d transform = futurePose.minus(hubPose);
        return Math.atan2(transform.getY(), transform.getX());
    }

    private int iterationCount = 3;

    public Command droneStrike() {
        return run(() -> {
            Pose2d currentPose = drivebase.getState().Pose;
            double hubDist = getDistFromHub(currentPose);
            Pose2d futurePose = new Pose2d();
            double airTime;
            double shootAngle;

            ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    drivebase.getState().Speeds,
                    currentPose.getRotation());

            for (int i = 0; i <= iterationCount; i++) {
                airTime = getTimeFromDist(hubDist);
                futurePose = getFuturePose(currentPose, velocity, airTime);
                hubDist = getDistFromHub(futurePose);
            }
            currentYaw = getYaw(futurePose);
            publisher.set(
                    new Pose3d(
                            futurePose.getX(),
                            futurePose.getY(),
                            0,
                            new Rotation3d(0, 0, currentYaw)));
            shootAngle = getAngleFromDist(hubDist);
            // hoodMotor.setHoodPosition(shootAngle);
            // Make it turn
            SmartDashboard.putNumber("shooterangle", shootAngle);
            double shotSpeed = 4;
            FuelSim.getInstance()
                    .spawnFuel(
                            new Translation3d(
                                    currentPose.getX(),
                                    currentPose.getY(),
                                    0),
                            new Translation3d(
                                    -shotSpeed *
                                            Math.cos(currentYaw) *
                                            Math.cos(shootAngle),
                                    -shotSpeed *
                                            Math.sin(currentYaw) *
                                            Math.cos(shootAngle),
                                    shotSpeed * Math.sin(shootAngle) * 2));
        });
    }

    // public void applyConfigs() {
    // slot0Configs.kP = Constants.shooterMotorKP;
    // slot0Configs.kI = Constants.shooterMotorKI;
    // slot0Configs.kD = Constants.shooterMotorKD;
    // frontShooter1.getConfigurator().apply(talonFXConfigs);
    // frontShooter2.getConfigurator().apply(talonFXConfigs);
    // frontShooter3.getConfigurator().apply(talonFXConfigs);
    // }

    public void setShooterState(ShooterStates newState) {
        shooterState = newState;
    }

    public ShooterStates getShooterState() {
        return shooterState;
    }

    @Override
    public void periodic() {
        tick += 1;
        SmartDashboard.putString("Shooter State", shooterState.toString());
        SmartDashboard.putNumber(
                "Front Shooter 1",
                frontShooter1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 1 Stator Current",
                frontShooter1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 1 Supply Current",
                frontShooter1.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 2",
                frontShooter2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 2 Stator Current",
                frontShooter2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 2 Supply Current",
                frontShooter2.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Front Shooter 3",
                frontShooter3.getVelocity().getValueAsDouble());
        SmartDashboard.getNumber("shooter/kp", Constants.shooterMotorKP);
        SmartDashboard.getNumber("shooter/ki", Constants.shooterMotorKI);
        SmartDashboard.getNumber("shooter/kd", Constants.shooterMotorKD);
        SmartDashboard.putNumber("shooter/velocity", calculateFeedingVelocity());
        SmartDashboard.putNumber("shooter/PoseTrench", poseFromTrench);

    }
}
