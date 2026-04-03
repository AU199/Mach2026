package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain.States;
import frc.robot.subsystems.IntakePivot.PivotStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class BusterAuto extends SequentialCommandGroup {

    private RobotContainer robotContainer;
    private SendableChooser<String> chooserAuto;
    private double angle;
    private CommandSwerveDrivetrain drivebase;
    private final PIDController pidControllerT = new PIDController(2.3, 0, 0.2);
    private final PIDController pidControllerR = new PIDController(4, 0, 0);
    private final PIDController pidControllerCT = new PIDController(2, 0, 0);
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public BusterAuto(
            RobotContainer robotContainer,
            SendableChooser<String> chooserAuto,
            CommandSwerveDrivetrain drivebase,
            IntakePivot intakePivot,
            IntakeRollers intakeRollers,
            Shooter shooter,
            // Hood hood,
            Feeder feeder) {
        this.robotContainer = robotContainer;
        this.chooserAuto = chooserAuto;
        this.drivebase = drivebase;
        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

        SmartDashboard.putBoolean("isRed", isRed);
        FollowPath.Builder pathBuilder = new FollowPath.Builder(
                drivebase,
                () -> drivebase.getState().Pose,
                () -> drivebase.getState().Speeds,
                speeds -> drivebase.setControl(autoRequest.withSpeeds(speeds)),
                pidControllerT,
                pidControllerR,
                pidControllerCT);

        switch (chooserAuto.getSelected()) {
            case "nothing":
                break;
            case "straight":
                if (isRed) {
                    angle = 0;
                } else {
                    angle = Math.PI;
                }
                addCommands(new PathPlannerAuto("One Meter Forward Auto"));
                break;
            case "CollectBallsTop":
                if (isRed) {
                    angle = 0;
                } else {
                    angle = Math.PI;
                }
                addCommands(
                        new GetAuto()
                                .blueTopCollectBalls(
                                        drivebase,
                                        intakePivot,
                                        intakeRollers,
                                        shooter,
                                        // hood,
                                        feeder));
                break;

            case "ShootMiddle":
                Translation2d midPose2d = drivebase.isAllianceRed().getAsBoolean()
                        ? FlippingUtil.flipFieldPosition(new Translation2d(3.7, 4))
                        : new Translation2d(3.7, 4);
                Path middlePath = new Path("middle_auto");
                middlePath.flip();
                if (isRed) {
                    angle = Math.PI;
                } else {
                    angle = 0;
                }
                addCommands(
                        new InstantCommand(() -> drivebase.resetPose(new Pose2d(midPose2d, new Rotation2d(0)))),
                        pathBuilder.build(middlePath),
                        // intakePivot.deploy().until(() -> intakePivot.getIntakeState() ==
                        // PivotStates.Deployed),
                        new ParallelCommandGroup(
                                shooter.shootFuel(),
                                // hood.shoot(),
                                feeder
                                        .feederOn(0)
                                        .until(
                                                () -> shooter
                                                        .getShooterState()
                                                        .equals(ShooterStates.Shooting) &&
                                                        drivebase.driveBaseState.equals(
                                                                States.InShootingPosition))
                                        .andThen(
                                                Commands.parallel(
                                                        Commands.waitSeconds(0.5).andThen(feeder.feederOn(1)),
                                                        Commands.waitSeconds(2)
                                                                .andThen(intakePivot.agitate())))));
                break;
            case "right":
                new InstantCommand(() -> drivetrain.resetPose(new Pose2d(rightPose2d, new Rotation2d(0)))),
                new ParallelCommandGroup(
                        intakePivot.deploy().until(() -> intakePivot.getIntakeState() == PivotStates.Deployed)
                        .andThen(new InstantCommand(() -> intakeRoller.setRollerSpeed(1))),
                                        pathBuilder.build(blueTopTrenchToTopOfBalls)),
                pathBuilder.build(blueTopBallsToBottomBalls),
                new InstantCommand(() -> intakeRoller.setRollerSpeed(0)),
                pathBuilder.build(blueBottomBallsToTopNeutralTrench),
                pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench),
                drivebase
                        .BlineToHub(1.778, 0.1, 0.1).deadlineFor(shooter.shootFuel()).andThen(
                                new ParallelCommandGroup(
                                        feeder
                                                .feederOn(0)
                                                .until(
                                                        () -> shooter
                                                                .getShooterState()
                                                                .equals(ShooterStates.Shooting) &&
                                                                drivebase.driveBaseState.equals(
                                                                        States.InShootingPosition))
                                                .andThen(feeder.feederOn(1).alongWith(intakePivot.agitate()))

                                ));
                break;
            case "left":
                if (isRed) {
                    angle = 0;
                } else {
                    angle = Math.PI;
                }
                addCommands(
                        new InstantCommand(() -> drivebase.resetRotation(new Rotation2d(angle))),
                        new PathPlannerAuto("3 coral auto left"));
                break;
        }
    }
}
