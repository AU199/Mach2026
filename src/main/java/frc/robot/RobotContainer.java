// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Commands.BusterAuto;
import frc.robot.Sotm.DroneStrike;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Levitator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.photon;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
        private double MaxSpeed = Constants.MaxDrivingSpeed; // kSpeedAt12Volts desired top
                                                             // speed
        private double MaxAngularRate = Constants.MaxAngularDrivingSpeed; // 3/4 of a rotation per
                                                                          // second max angular
                                                                          // velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final Field2d m_field = new Field2d();
        private final CommandPS4Controller controller1 = new CommandPS4Controller(0);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Shooter shooter = new Shooter(drivetrain, true, m_field);
        public final Hood hood = new Hood();
        public final Levitator levitator = new Levitator();
        public final Intake intake = new Intake();
        public final Feeder feeder = new Feeder(intake);
        public final Limelight limelight = new Limelight(drivetrain);
        // public final photon photon = new photon(drivetrain);
        // public final Shooter shooter = new Shooter(drivetrain,true,m_field);
        private SendableChooser<String> chooserAuto = new SendableChooser<String>();
        private final double x = 2.362, y = 5.205;
        private final Pose2d targetPoseHubLeft = new Pose2d(x, y,
                        new Rotation2d(Constants.blueHubPose.getX() - x,
                                        Constants.blueHubPose.getY() - y).plus(new Rotation2d(Math.PI)));
        private final Pose2d targetPoseHubRight = new Pose2d(x, y,
                        new Rotation2d(Constants.blueHubPose.getX() - x,
                                        Constants.blueHubPose.getY() - (8 - y))
                                        .plus(new Rotation2d((3 * Math.PI) / 2)));
        private final Pose2d targetPoseTrenchLeft = new Pose2d(6, 7.4, new Rotation2d(0));
        private final Pose2d targetPoseTrenchRight = new Pose2d(6, 0.6, new Rotation2d(0));

        public RobotContainer() {
                chooserAuto.addOption("nothing", "nothing");
                chooserAuto.addOption("Straight 1m", "straight");
                chooserAuto.addOption("CollectBallsTop", "CollectBallsTop");

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
                controller1.share().onTrue(Commands.sequence(
                                // intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5).withTimeout(2),
                                intake.runRoller(.5).withTimeout(2),
                                // intake.setIntakePosition(Constants.IntakeRetractPos, 0.025,
                                // 0.3).withTimeout(5),
                                hood.setHoodPosition(0.2).withTimeout(1),
                                shooter.shooterOn(1).withTimeout(2),
                                hood.setHoodPosition(0).withTimeout(1),
                                feeder.feederOn(1).withTimeout(2),
                                levitator.lift().withTimeout(2),
                                levitator.retract().withTimeout(2)));

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-Math.pow(controller1.getRawAxis(1), 3) * MaxSpeed) // Drive
                                                                                                                   // forward
                                                // with negative Y
                                                // (forward)
                                                .withVelocityY(-Math.pow(controller1.getRawAxis(0), 3) * MaxSpeed) // Drive
                                                                                                                   // left
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // X
                                                                                                                   // (left)
                                                .withRotationalRate(-controller1.getRawAxis(2) * MaxAngularRate) // Drive
                                                                                                                 // counterclockwise
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // X
                                                                                                                 // (left)
                                ));

                controller1.cross().whileTrue(intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5));
                controller1.triangle().whileTrue(intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3));
                controller1.R1().whileTrue(intake.runRoller(1));
                // controller1.square().whileTrue(intake.runRoller(0.35));
                controller1.L2().whileTrue(shooter.shooterOn(50).alongWith(hood.setHoodPosition(0.10)));
                controller1.L1().whileTrue(shooter.shooterOn(50).alongWith(hood.setHoodPosition(0.11)));
                // controller1.cross().whileTrue(shooter.shooterOn(50));
                controller1.R2().whileTrue(feeder.feederOn(1));

                controller1.options().onTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))));

                controller1.povUp().whileTrue(levitator.runLevitator(1));
                controller1.povDown().whileTrue(levitator.runLevitator(-1));

                controller1.circle().toggleOnTrue(
                                drivetrain.BlineToPoint(targetPoseHubLeft, targetPoseHubRight, 1.90, 2.40, 0).alongWith(
                                                shooter.shooterOn(50).alongWith(hood.setHoodPosition(0.10))));
                controller1.square().toggleOnTrue(
                                drivetrain.BlineToPoint(targetPoseTrenchLeft, targetPoseTrenchRight, 6, 2, 1));

                // controller1.circle().onTrue(new InstantCommand(() ->
                // shooter.applyConfigs()));

                // controller1.L1().whileTrue(new DroneStrike(drivetrain, Constants.blueHubPose,
                // hood, shooter, feeder, Constants.ballInitialVelocityFromShooterHub,
                // Constants.ballInitialSpinFromShooterHub, () -> controller1.getRawAxis(1), ()
                // -> controller1.getRawAxis(0)));

                // controller1.L2().whileTrue(new InstantCommand(() ->
                // FuelSim.getInstance().clearFuel()));

                // Feeding balls into alliance zone

                // Aim robot at blue hub
                // controller1.triangle().whileTrue(drivetrain.pidToRotation(Math.atan2(Constants.blueHubPose.getY()
                // - drivetrain.getState().Pose.getY(), Constants.blueHubPose.getX() -
                // drivetrain.getState().Pose.getX())-Robot.phiPassing, () ->
                // controller1.getRawAxis(0),() -> controller1.getRawAxis(1)));

                // controller1.circle().whileTrue(drivetrain.pidToRotation(Robot.phiPassing, ()
                // -> controller1.getRawAxis(0),() -> controller1.getRawAxis(1)));
                // controller1.square()
                // .whileTrue(drivetrain.pidToPoint(
                // new Pose2d(1.728, 6.826,
                // new Rotation2d(Constants.blueHubPose.getX() - 1.728,
                // Constants.blueHubPose.getY() - 6.826).plus(new Rotation2d(Math.PI))),
                // () -> DriverStation.getAlliance().get().equals(Alliance.Red)));
                // controller1.povUp().onTrue(hood.setHoodPosition(hood.getHoodAngleDash().getAsDouble()));
                // controller1.povDown().onTrue(hood.setHoodPosition(0.11));
                // controller1.povRight().onTrue(hood.setHoodPosition(0.09));
                // controller1.povLeft().onTrue(hood.setHoodPosition(0.07));

                NamedCommands.registerCommand("DeployIntake",
                                intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5).withTimeout(3));
                NamedCommands.registerCommand("RetractIntake",
                                intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3));
                NamedCommands.registerCommand("StartBallIntake", new InstantCommand(() -> intake.setRollerSpeed(0.45)));
                NamedCommands.registerCommand("EndBallIntake", new InstantCommand(() -> intake.setRollerSpeed(0)));
                NamedCommands.registerCommand("ShootBalls",
                                hood.setHoodPosition(0.09).andThen(
                                                shooter.shooterOn(60).withTimeout(2).andThen(feeder.feederOn(1))));
                // NamedCommands.registerCommand("PIDToShoot",
                // drivetrain.pidToPoint(
                // new Pose2d(1.728, 6.826,
                // new Rotation2d(Constants.blueHubPose.getX() - 1.728,
                // Constants.blueHubPose.getY() - 6.826).plus(new Rotation2d(Math.PI))),
                // () ->
                // DriverStation.getAlliance().get().equals(Alliance.Red)).withTimeout(3));
                NamedCommands.registerCommand("StopMoving", drivetrain.applyRequest(() -> new SwerveRequest.Idle()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                return new BusterAuto(this, this.chooserAuto, drivetrain, intake, shooter, hood, feeder);
        }


        public Hood getHood(){
                return hood;
        }
        public Intake getIntake(){
                return intake;
        }
        public Shooter getShooter(){
                return shooter;
        }
        public Feeder getFeeder(){
                return feeder;
        }
}
