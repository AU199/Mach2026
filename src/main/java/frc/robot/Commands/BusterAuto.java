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
    private final Path middlePath = new Path("middle_auto");
    private final Path blueTopBallsToBottomBalls = new Path("BlueTopBallsToBottomBalls");
    private final Path blueTopNeutralTrenchToTopBlueTrench = new Path(
            "BlueTopNeutralTrenchToTopBlueTrench");
    private final Path blueBottomBallsToTopNeutralTrench = new Path(
            "BlueBottomBallsToTopNeutralTrench");
    private final Path blueTopTrenchToTopOfBalls = new Path("BlueTopTrenchToTopOfBalls");

    private void flipPaths() {
        middlePath.flip();
        blueTopBallsToBottomBalls.flip();
        blueTopNeutralTrenchToTopBlueTrench.flip();
        blueBottomBallsToTopNeutralTrench.flip();
        blueTopTrenchToTopOfBalls.flip();
    }

    private void mirrorPaths() {
        middlePath.mirror();
        blueTopBallsToBottomBalls.mirror();
        blueTopNeutralTrenchToTopBlueTrench.mirror();
        blueBottomBallsToTopNeutralTrench.mirror();
        blueTopTrenchToTopOfBalls.mirror();
    }

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
                if (isRed) {
                    angle = 0;
                    flipPaths();
                } else {
                    angle = Math.PI;
                }
                addCommands(
                        new InstantCommand(() -> drivebase.resetPose(new Pose2d(midPose2d, new Rotation2d(angle)))),
                        // pathBuilder.build(middlePath),
                        intakePivot.deploy().until(() -> intakePivot.getIntakeState() == PivotStates.Deployed),
                        new ParallelCommandGroup(
                                drivebase.BlineToHub(1.778, 0.1, 0.1),
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
                                                        drivebase.driveBaseState
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
                                                                        .agitate())))));
                break;
            case "right":
                Translation2d rightPose2d = drivebase.isAllianceRed().getAsBoolean()
                        ? FlippingUtil.flipFieldPosition(new Translation2d(4.414, 8.042656 - 7.486))
                        : new Translation2d(4.414, 8.042656 - 7.486);
                mirrorPaths();
                if (isRed) {
                    angle = Math.PI;
                    flipPaths();
                } else {
                    angle = 0;
                }
                addCommands(
                        new InstantCommand(() -> drivebase.resetPose(new Pose2d(rightPose2d, new Rotation2d(angle)))),
                        new ParallelCommandGroup(
                                intakePivot.deploy().until(() -> intakePivot.getIntakeState() == PivotStates.Deployed)
                                        .andThen(new InstantCommand(() -> intakeRollers.setRollerSpeed(1))),
                                pathBuilder.build(blueTopTrenchToTopOfBalls)),
                        pathBuilder.build(blueTopBallsToBottomBalls),
                        new InstantCommand(() -> intakeRollers.setRollerSpeed(0)),
                        pathBuilder.build(blueBottomBallsToTopNeutralTrench),
                        pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench),
                        new ParallelCommandGroup(
                                drivebase.BlineToHub(1.778, 0.1, 0.1),
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
                                                        drivebase.driveBaseState
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
                                                                        .agitate())))));
                break;
            case "left":

                if (isRed) {
                    angle = Math.PI;
                    flipPaths();
                } else {
                    angle = 0;
                }
                Translation2d leftPose2d = drivebase.isAllianceRed().getAsBoolean()
                        ? FlippingUtil.flipFieldPosition(new Translation2d(4.414, 7.486))
                        : new Translation2d(4.414, 7.486);

                addCommands(
                        new InstantCommand(() -> drivebase.resetPose(new Pose2d(leftPose2d, new Rotation2d(angle)))),
                        new ParallelCommandGroup(
                                intakePivot.deploy().until(() -> intakePivot.getIntakeState() == PivotStates.Deployed)
                                        .andThen(new InstantCommand(() -> intakeRollers.setRollerSpeed(1))),
                                pathBuilder.build(blueTopTrenchToTopOfBalls)),
                        pathBuilder.build(blueTopBallsToBottomBalls),
                        new InstantCommand(() -> intakeRollers.setRollerSpeed(0)),
                        pathBuilder.build(blueBottomBallsToTopNeutralTrench),
                        pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench),
                        new ParallelCommandGroup(
                                drivebase.BlineToHub(1.778, 0.1, 0.1),
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
                                                        drivebase.driveBaseState
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
                                                                        .agitate())))));
                break;
        }
    }
}
