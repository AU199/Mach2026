package frc.robot.Sotm;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FuelSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class DroneStrike extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    private Hood hoodMotor;
    private Shooter shooter;
    private Feeder feeder;

    private Supplier<Double> controllerXAxis;
    private Supplier<Double> controllerYAxis;

    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
    private double MaxAngularRate = 1 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second max angular
                                                                                          // velocity

    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private double theta;
    private double phi;

    private double ballInitialSpeedFromShooter;
    private double ballInitialSpinFromShooter;

    private double shootingTick = 0;

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose3d.struct)
            .publish();

    public DroneStrike(CommandSwerveDrivetrain drivetrain, Pose2d targetPose,
            Hood hoodMotor, Shooter shooter, Feeder feeder,
            double ballInitialSpeedFromShooter, double ballInitialSpinFromShooter,
            Supplier<Double> controllerXAxis, Supplier<Double> controllerYAxis) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.hoodMotor = hoodMotor;
        this.shooter = shooter;
        this.feeder = feeder;
        this.ballInitialSpeedFromShooter = ballInitialSpeedFromShooter;
        this.ballInitialSpinFromShooter = ballInitialSpinFromShooter;

        this.controllerXAxis = controllerXAxis;
        this.controllerYAxis = controllerYAxis;
        addRequirements(drivetrain, hoodMotor, shooter, feeder);
    }

    @Override
    public void initialize() {
    }

    // spinDirection: 1.0 = backspin, -1.0 = topspin
    @Override
    public void execute() {
        shootingTick++;

        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds robotRobotRelativeVelocity = drivetrain.getState().Speeds;
        ChassisSpeeds robotFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRobotRelativeVelocity,
                robotPose.getRotation());

        IdealTrajectory idealTrajectory = new IdealTrajectory(robotPose, targetPose, robotFieldRelativeVelocity,
                ballInitialSpeedFromShooter);
        ShotAngles currentAngles = idealTrajectory.getIdealShotAngles();

        theta = currentAngles.getTheta();
        phi = currentAngles.getPhi();

        SmartDashboard.putNumber("Ideal Theta", theta);
        SmartDashboard.putNumber("Ideal Phi", phi);

        Newton newton = new Newton(robotPose, targetPose, robotFieldRelativeVelocity, ballInitialSpeedFromShooter,
                ballInitialSpinFromShooter);

        ShotAngles anglesFromNewton = newton.findOptimalTrajectory(currentAngles);
        if (!(Double.isNaN(anglesFromNewton.getTheta()) || Double.isNaN(anglesFromNewton.getPhi()))) {
            System.out.println("Newton returned");
            currentAngles = anglesFromNewton;
        } else {
            System.out.println("Shot was NaN");

            // Ends but with no rotation
            feeder.feederOn(0);
            drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(controllerXAxis.get(), 3) * MaxSpeed)
                .withVelocityY(-Math.pow(controllerYAxis.get(), 3) * MaxSpeed)).execute();
            return;
        }

        Vector ballLinearVelocity = newton.getFinalBallLinearVelocity();
        if (ballLinearVelocity == null) {
            System.out.println("Ball Velocity Null");

            // Ends but with no rotation
            feeder.feederOn(0);
            drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(controllerXAxis.get(), 3) * MaxSpeed)
                .withVelocityY(-Math.pow(controllerYAxis.get(), 3) * MaxSpeed)).execute();
            return;
        }

        theta = currentAngles.getTheta();
        phi = currentAngles.getPhi();
        SmartDashboard.putNumber("Newton Theta", theta);
        SmartDashboard.putNumber("Newton Phi", phi);

        hoodMotor.setHoodPosition(theta);
        publisher.set(new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, phi)));

        SmartDashboard.putNumber("ControllerXAxis", controllerXAxis.get());
        SmartDashboard.putNumber("ControllerYAxis", controllerYAxis.get());

        shooter.shooterOn(MaxSpeed).execute();

        drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(controllerXAxis.get(), 3) * MaxSpeed) // Drive
                                                                                                          // forward
                // with negative Y
                // (forward)
                .withVelocityY(-Math.pow(controllerYAxis.get(), 3) * MaxSpeed) // Drive left with negative X (left)
                .withTargetDirection(new Rotation2d(phi)) // Drive counterclockwise with negative X (left)
                .withHeadingPID(20, 0, 0)
                .withMaxAbsRotationalRate(MaxAngularRate)).execute();

        // Spawn fuel ball in FuelSim with velocity from shot angles

        if (shootingTick % 20 == 0) {
            FuelSim.getInstance().spawnFuel(
                new Translation3d(robotPose.getX(), robotPose.getY(), Constants.shooterHeight),
                new Translation3d(ballLinearVelocity));
        }
    }

    @Override
    public void end(boolean interupted) {
    }
}
