// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Commands.BusterAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.States;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakePivot.PivotStates;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Levitator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import java.util.function.DoubleSupplier;

public class RobotContainer {

        private double MaxSpeed = Constants.MaxDrivingSpeed; // kSpeedAt12Volts desired top
        // speed
        private double MaxAngularRate = Constants.MaxAngularDrivingSpeed; // 3/4 of a rotation per
        // second max angular
        // velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
        // motors
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final Field2d m_field = new Field2d();
        private final CommandPS4Controller controller1 = new CommandPS4Controller(
                        0);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Shooter shooter = new Shooter(drivetrain, true, m_field);
        // public final Hood hood = new Hood();
        public final Levitator levitator = new Levitator();
        public final IntakePivot intakePivot = new IntakePivot();
        public final IntakeRollers intakeRollers = new IntakeRollers();
        public final Feeder feeder = new Feeder();
        public final Limelight limelight = new Limelight(drivetrain);
        // public final photon photon = new photon(drivetrain);
        // public final Shooter shooter = new Shooter(drivetrain,true,m_field);
        private SendableChooser<String> chooserAuto = new SendableChooser<String>();
        private final SlewRateLimiter driverControllerSlewRateLimiterX = new SlewRateLimiter(15);
        private final SlewRateLimiter driverControllerSlewRateLimiterY = new SlewRateLimiter(15);
        private final DoubleSupplier driveX;
        private final DoubleSupplier driveY;

        public RobotContainer() {
                chooserAuto.addOption("nothing", "nothing");
                chooserAuto.addOption("Straight 1m", "straight");
                chooserAuto.addOption("CollectBallsTop", "CollectBallsTop");
                chooserAuto.addOption("ShootMiddle", "ShootMiddle");
                chooserAuto.addOption("right", "right");
                chooserAuto.addOption("left", "left");

                driveX = () -> driverControllerSlewRateLimiterX.calculate(
                                -Math.pow(controller1.getRawAxis(1), 3) * MaxSpeed);
                driveY = () -> driverControllerSlewRateLimiterY.calculate(
                                -Math.pow(controller1.getRawAxis(0), 3) * MaxSpeed);

                FuelSim.getInstance();
                FuelSim.getInstance().start();
                configureBindings();
                SmartDashboard.putData(chooserAuto);
        }

        private void configureBindings() {
                // controller1.a().whileTrue(shooter.shooterOn(1));
                // controller1.b().whileTrue(feeder.feederOn(1));
                // controller1.leftBumper().whileTrue(shooter.pivotMotorOn(.25));
                // controller1.rightBumper().whileTrue(shooter.pivotMotorOn(-.25));
                controller1
                                .share()
                                .whileTrue(
                                                Commands.sequence(
                                                                Commands.either(
                                                                                // Already deployed or in depot — just
                                                                                // run rollers
                                                                                Commands.startEnd(
                                                                                                () -> intakeRollers
                                                                                                                .setRollerSpeed(1),
                                                                                                () -> intakeRollers
                                                                                                                .setRollerSpeed(0)),
                                                                                // Not deployed — deploy first, then run
                                                                                // rollers
                                                                                intakePivot
                                                                                                .deploy()
                                                                                                .andThen(
                                                                                                                Commands.startEnd(
                                                                                                                                () -> intakeRollers
                                                                                                                                                .setRollerSpeed(1),
                                                                                                                                () -> intakeRollers
                                                                                                                                                .setRollerSpeed(0))),
                                                                                // condition: either deployed or in
                                                                                // depot
                                                                                () -> intakePivot
                                                                                                .getIntakeState()
                                                                                                .equals(PivotStates.Deployed)
                                                                                                ||
                                                                                                intakePivot.getIntakeState()
                                                                                                                .equals(PivotStates.Depot)),

                                                                shooter.shooterOn(Constants.shootingSpeed)
                                                                                .withTimeout(2),
                                                                feeder.feederOn(1).withTimeout(2)));

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive
                                                                .withVelocityX(
                                                                                driveX.getAsDouble()) // Drive
                                                                                                      // forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
                                                                .withVelocityY(
                                                                                driveY.getAsDouble()) // Drive
                                                                                                      // left
                                                                                                      // with
                                                                                                      // negative
                                                                                                      // X
                                                                                                      // (left)
                                                                .withRotationalRate(
                                                                                -controller1.getRawAxis(2)
                                                                                                * MaxAngularRate) // Drive
                                // counterclockwise
                                // with
                                // negative
                                // X
                                // (left)
                                ));

                controller1.cross().toggleOnTrue(intakePivot.deploy());
                controller1.square().whileTrue(feeder.feederOn(1))
                                .onFalse(feeder.feederOn(0));
                controller1.triangle().toggleOnTrue(intakePivot.retract());

                controller1
                                .R1()
                                .whileTrue(
                                                Commands.either(
                                                                // Already deployed or in depot — just run rollers
                                                                Commands.startEnd(
                                                                                () -> intakeRollers.setRollerSpeed(1),
                                                                                () -> intakeRollers.setRollerSpeed(0)),
                                                                // Not deployed — deploy first, then run rollers
                                                                intakePivot
                                                                                .deploy()
                                                                                .andThen(
                                                                                                Commands.startEnd(
                                                                                                                () -> intakeRollers
                                                                                                                                .setRollerSpeed(1),
                                                                                                                () -> intakeRollers
                                                                                                                                .setRollerSpeed(0))),
                                                                // condition: either deployed or in depot
                                                                () -> intakePivot
                                                                                .getIntakeState()
                                                                                .equals(PivotStates.Deployed) ||
                                                                                intakePivot.getIntakeState().equals(
                                                                                                PivotStates.Depot)));

                controller1
                                .povUp()
                                .whileTrue(
                                                Commands.either(
                                                                // Already deployed or in depot — just run rollers
                                                                Commands.startEnd(
                                                                                () -> intakeRollers.setRollerSpeed(-1),
                                                                                () -> intakeRollers.setRollerSpeed(0)),
                                                                // Not deployed — deploy first, then run rollers
                                                                intakePivot
                                                                                .deploy()
                                                                                .andThen(
                                                                                                Commands.startEnd(
                                                                                                                () -> intakeRollers
                                                                                                                                .setRollerSpeed(-1),
                                                                                                                () -> intakeRollers
                                                                                                                                .setRollerSpeed(0))),
                                                                // condition: either deployed or in depot
                                                                () -> intakePivot
                                                                                .getIntakeState()
                                                                                .equals(PivotStates.Deployed) ||
                                                                                intakePivot.getIntakeState().equals(
                                                                                                PivotStates.Depot)));

                // controller1.povLeft().on(shooter.increasingSpeed());

                controller1.L1().whileTrue(shooter.shootFuel())
                                .onFalse(shooter.idle());

                controller1.L2().whileTrue(shooter.feedFuel())
                                .onFalse(shooter.idle());

                // controller1
                // .L2()
                // .toggleOnTrue(
                // shooter.feedFuel().alongWith(hood.feed())
                // )
                // .toggleOnFalse(
                // shooter.idle().alongWith(hood.idle())
                // );

                controller1
                                .R2()
                                .whileTrue(
                                                Commands.either(
                                                                // shoot
                                                                new ParallelCommandGroup(
                                                                                drivetrain.BlineToHub(1.778, 0.1, 0.1),
                                                                                shooter.shootFuel(),
                                                                                // hood.shoot(),
                                                                                // intakePivot.deploy(),
                                                                                feeder
                                                                                                .feederOn(0)
                                                                                                .until(
                                                                                                                () -> shooter
                                                                                                                                .getShooterState()
                                                                                                                                .equals(ShooterStates.Shooting)
                                                                                                                                &&
                                                                                                                                drivetrain.driveBaseState
                                                                                                                                                .equals(
                                                                                                                                                                States.InShootingPosition))
                                                                                                .andThen(
                                                                                                                Commands.parallel(
                                                                                                                                Commands.waitSeconds(
                                                                                                                                                0.5)
                                                                                                                                                .andThen(feeder.feederOn(
                                                                                                                                                                1)),
                                                                                                                                Commands.waitSeconds(
                                                                                                                                                2)
                                                                                                                                                .andThen(intakePivot
                                                                                                                                                                .agitate())))),
                                                                // feed
                                                                new ParallelCommandGroup(
                                                                                drivetrain.pidToRotation(0, driveX,
                                                                                                driveY),
                                                                                shooter.feedFuel(),
                                                                                // intakePivot.deploy(),
                                                                                // hood.feed(),
                                                                                feeder
                                                                                                .feederOn(0)
                                                                                                .until(
                                                                                                                () -> shooter
                                                                                                                                .getShooterState()
                                                                                                                                .equals(ShooterStates.Feeding)
                                                                                                                                &&
                                                                                                                                drivetrain.driveBaseState
                                                                                                                                                .equals(
                                                                                                                                                                States.Feeding))
                                                                                                .andThen(
                                                                                                                Commands.parallel(
                                                                                                                                Commands.waitSeconds(
                                                                                                                                                0.5)
                                                                                                                                                .andThen(feeder.feederOn(
                                                                                                                                                                1)),
                                                                                                                                Commands.waitSeconds(
                                                                                                                                                2)
                                                                                                                                                .andThen(intakePivot
                                                                                                                                                                .agitate()))

                                                                                                )),
                                                                () -> drivetrain.checkIfInSameAlliance(
                                                                                drivetrain.isAllianceRed()
                                                                                                .getAsBoolean())))
                                .onFalse(
                                                new InstantCommand(() -> feeder.feederIdle()).alongWith(
                                                                intakePivot.deploy(),
                                                                shooter.idle(),
                                                                Commands.runOnce(
                                                                                () -> drivetrain.driveBaseState = States.Driven)));

                controller1
                                .options()
                                .onTrue(
                                                new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))));

                controller1.povUp().whileTrue(levitator.runLevitator(1));
                controller1.povDown().whileTrue(levitator.runLevitator(-1));

                controller1.circle().whileTrue(drivetrain.BlineToTrench());

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                return new BusterAuto(
                                this,
                                this.chooserAuto,
                                drivetrain,
                                intakePivot,
                                intakeRollers,
                                shooter,
                                // hood,
                                feeder);
        }
}
