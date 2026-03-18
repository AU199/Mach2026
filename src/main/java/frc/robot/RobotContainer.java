// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.proto.Photon;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.Commands.BusterAuto;
import frc.robot.Sotm.DroneStrike;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Levitator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.photon;

public class RobotContainer {
    private double MaxSpeed = Constants.MaxDrivingSpeed; // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = Constants.MaxAngularDrivingSpeed; // 3/4 of a rotation per
                                                                                            // second max angular
                                                                                            // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Field2d m_field = new Field2d();
    private final CommandPS4Controller controller1 = new CommandPS4Controller(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter(drivetrain, true, m_field);
    public final Hood hood = new Hood();
    public final Levitator levitator = new Levitator();
    public final Intake intake = new Intake();
    public final Feeder feeder = new Feeder();
    public final photon photon = new photon(drivetrain);
    // public final Shooter shooter = new Shooter(drivetrain,true,m_field);

    public RobotContainer() {
        FuelSim.getInstance();
        FuelSim.getInstance().start();
        configureBindings();
    }

    private void configureBindings() {
    //     controller1.a().whileTrue(shooter.shooterOn(1));
    //     controller1.b().whileTrue(feeder.feederOn(1));
    //     controller1.leftBumper().whileTrue(shooter.pivotMotorOn(.25));
    //     controller1.rightBumper().whileTrue(shooter.pivotMotorOn(-.25));
        controller1.share().onTrue(Commands.sequence(
            // intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5).withTimeout(2),
            intake.runRoller(.5).withTimeout(2),
            // intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3).withTimeout(5),
            hood.setHoodPosition(0.2).withTimeout(1),
            shooter.shooterOn(1).withTimeout(2),
            hood.setHoodPosition(0).withTimeout(1),
            feeder.feederOn(1).withTimeout(2),
            levitator.lift().withTimeout(2),
            levitator.retract().withTimeout(2)
        ));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(controller1.getRawAxis(1), 3) * MaxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
                        .withVelocityY(-Math.pow(controller1.getRawAxis(0), 3) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller1.getRawAxis(2) * MaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
                ));

        controller1.cross().whileTrue(intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5));
        controller1.triangle().whileTrue(intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3));
        // controller1.circle().whileTrue(intake.runPivotSetSpeed(0.1));
        // controller1.square().whileTrue(intake.runPivotSetSpeed(-0.1));
        controller1.R1().whileTrue(intake.runRoller(0.5));
        // controller1.square().whileTrue(intake.runRoller(0.35));
        controller1.R2().whileTrue(shooter.shooterOn(7));
        controller1.L1().whileTrue(shooter.shooterOn(80));
        // controller1.cross().whileTrue(shooter.shooterOn(50));
        controller1.L2().whileTrue(feeder.feederOn(1));

        controller1.options().onTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))));

        controller1.circle().onTrue(new InstantCommand(() -> shooter.applyConfigs()));

    
        // controller1.L1().whileTrue(new DroneStrike(drivetrain, Constants.blueHubPose, hood, Constants.ballInitialVelocityFromShooterHub, Constants.ballInitialSpinFromShooterHub, () -> controller1.getRawAxis(1), () -> controller1.getRawAxis(0)));

        // Feeding balls into alliance zone


        // controller1.povUp().whileTrue(levitator.runLevitator(1));
        // controller1.povDown().whileTrue(levitator.runLevitator(-1));
        controller1.share().whileTrue(drivetrain.pidToRotation(Robot.phiPassing, () -> controller1.getRawAxis(0),() -> controller1.getRawAxis(1)));
        controller1.square().whileTrue(drivetrain.pidToPoint(new Pose2d(2.8, 4, new Rotation2d(0))));

        controller1.povUp().onTrue(hood.setHoodPosition(hood.getHoodAngleDash().getAsDouble()));
        controller1.povDown().onTrue(hood.setHoodPosition(0.12));
        controller1.povRight().onTrue(hood.setHoodPosition(0.13));
        controller1.povLeft().onTrue(hood.setHoodPosition(0.14));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
