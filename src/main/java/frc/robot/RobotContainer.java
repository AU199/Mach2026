// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Sotm.DroneStrike;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Levitator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;

public class RobotContainer {
    private double MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = 0.6 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
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
            shooter.shooterOn(0.75).withTimeout(2),
            // hood.setHoodPosition(0.5).withTimeout(1),
            // hood.setHoodPosition(0).withTimeout(1),
            intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5).withTimeout(2),
            intake.runRoller(.2).withTimeout(2),
            intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3).withTimeout(10),
            levitator.lift().withTimeout(2),
            levitator.retract().withTimeout(2),
            feeder.feederOn(0.1).withTimeout(2)
        ));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(controller1.getRawAxis(1) * MaxSpeed) // Drive forward
                                                                                                     // with negative Y
                                                                                                     // (forward)
                        .withVelocityY(controller1.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-controller1.getRawAxis(2) * MaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
                ));

        controller1.cross().whileTrue(intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5));
        controller1.triangle().whileTrue(intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3));
        // controller1.circle().whileTrue(intake.runPivotSetSpeed(0.1));
        // controller1.square().whileTrue(intake.runPivotSetSpeed(-0.1));
        controller1.circle().whileTrue(intake.runRoller(0.35));
        controller1.R1().whileTrue(intake.runRoller(0.3));
        controller1.R2().whileTrue(shooter.shooterOn(0.6));
        controller1.L2().whileTrue(feeder.feederOn(1));


        controller1.povUp().whileTrue(hood.setHoodPosition(-0.25));
        controller1.povDown().whileTrue(hood.setHoodPosition(0.25));
        controller1.povRight().whileTrue(hood.setHoodPosition(0));
        controller1.povLeft().whileTrue(hood.setHoodPosition(Constants.hoodHardStopAngle));


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
