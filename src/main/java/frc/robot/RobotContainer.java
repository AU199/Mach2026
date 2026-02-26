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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HangArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

public class RobotContainer {
    private double MaxSpeed = 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = 0.6 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                            // second max angular
                                                                                            // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.04).withRotationalDeadband(MaxAngularRate * 0.04) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Field2d m_field = new Field2d();
    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter(drivetrain, true, m_field);
    public final HangArm hangArm = new HangArm();
    public final Intake intake = new Intake();
    public final Feeder feeder = new Feeder();
    // public final Shooter shooter = new Shooter(drivetrain,true,m_field);

    public RobotContainer() {
        FuelSim.getInstance();
        FuelSim.getInstance().start();
        configureBindings();
    }

    private void configureBindings() {
        controller1.a().whileTrue(shooter.shooterOn());
        controller1.leftBumper().whileTrue(shooter.pivotMotorOn(.25));
        controller1.rightBumper().whileTrue(shooter.pivotMotorOn(-.25));


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
        // controller1.a().whileTrue(intake.runRoller());
        //controller1.b().whileTrue();
        // controller1.x().whileTrue(hangArm.runHangArm(-1));
        // controller1.y().whileTrue(hangArm.runHangArm(1));

        controller1.x().whileTrue(shooter.droneStrikeRK4());
        controller1.y().onTrue(new InstantCommand(() -> FuelSim.getInstance().clearFuel()));

        // controller2.a().whileTrue(shooter.shooterOn());
        // controller2.b().whileTrue();
        controller1.a().whileTrue(feeder.feederOn(0.7));
        controller1.b().whileTrue(feeder.feederOn(-0.7));

        // controller1.back().onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
        // drivetrain.setDefaultCommand(
        //         // Drivetrain will execute this command periodically
        //         drivetrain.applyRequest(() -> drive.withVelocityX(controller1.getRawAxis(1) * MaxSpeed) // Drive forward
        //                                                                                              // with negative Y
        //                                                                                              // (forward)
        //                 .withVelocityY(controller1.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
        //                 .withRotationalRate(-controller1.getRawAxis(2) * MaxAngularRate) // Drive counterclockwise with
        //                                                                               // negative X (left)
        //         ));
        //controller1.a().whileTrue(intake.runRoller());
        //controller1.b().whileTrue();
        //controller1.x().whileTrue(hangArm.runHangArm(-1));
        //controller1.y().whileTrue(hangArm.runHangArm(1));

        //controller2.a().whileTrue(shooter.shooterOn());
        // controller2.b().whileTrue();
        // controller2.x().whileTrue(feeder.feederOn(0.7));
        //controller2.y().whileTrue(feeder.feederOn(-0.7));

       // controller1.back().onTrue(new InstantCommand(() -> drivetrain.zeroGyro()));
        // drive with speed at .5 while left trigger is held, for testing
        // controller1.leftTrigger(.5).whileTrue(
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(joystick.getRawAxis(1) * MaxSpeed * 0.5)
        // .withVelocityY(joystick.getRawAxis(0) * MaxSpeed * 0.5)
        // .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate * 0.5)
        // )
        // );

        //controller2.leftBumper().onTrue(intake.deployIntake());
        //controller2.rightBumper().onTrue(intake.retractIntake());
        

       // controller2.leftTrigger(.5).onTrue(shooter.droneStrike());
        // controller2.rightTrigger(.5).onTrue();
     //   controller2.leftBumper().onTrue(shooter.setPivotAngle(0.5));
   //     controller2.rightBumper().onTrue(shooter.setPivotAngle(0));

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
