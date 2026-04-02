package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.States;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakePivot.PivotStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class GetAuto {

    private final PIDController pidControllerT = new PIDController(2.3, 0, 0.2);
    private final PIDController pidControllerR = new PIDController(4, 0, 0);
    private final PIDController pidControllerCT = new PIDController(2, 0, 0);
    private final double x = 2.362, y = 5.205;
    private final Pose2d targetPoseHubLeft = new Pose2d(
            x,
            y,
            new Rotation2d(
                    Constants.blueHubPose.getX() - x,
                    Constants.blueHubPose.getY() - y).plus(new Rotation2d(Math.PI)));
    private final Pose2d targetPoseHubRight = new Pose2d(
            x,
            y,
            new Rotation2d(
                    Constants.blueHubPose.getX() - x,
                    Constants.blueHubPose.getY() - (8 - y)).plus(new Rotation2d((3 * Math.PI) / 2)));
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public Command blueTopCollectBalls(
            CommandSwerveDrivetrain drivetrain,
            IntakePivot intakePivot,
            IntakeRollers intakeRoller,
            Shooter shooter,
            Hood hood,
            Feeder feeder) {
        pidControllerR.setTolerance(0.75);
        pidControllerT.setTolerance(0.75);
        pidControllerCT.setTolerance(0.75);

        FollowPath.Builder pathBuilder = new FollowPath.Builder(
                drivetrain,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                speeds -> drivetrain.setControl(autoRequest.withSpeeds(speeds)),
                pidControllerT,
                pidControllerR,
                pidControllerCT);

        Path blueTopTrenchToTopOfBalls = new Path("BlueTopTrenchToTopOfBalls");
        blueTopTrenchToTopOfBalls.mirror();
        Path blueTopBallsToBottomBalls = new Path("BlueTopBallsToBottomBalls");
        blueTopBallsToBottomBalls.mirror();
        Path blueBottomBallsToTopNeutralTrench = new Path(
                "BlueBottomBallsToTopNeutralTrench");
        blueBottomBallsToTopNeutralTrench.mirror();
        Path blueTopNeutralTrenchToTopBlueTrench = new Path(
                "BlueTopNeutralTrenchToTopBlueTrench");
        blueTopNeutralTrenchToTopBlueTrench.mirror();

        return Commands.sequence(
                new InstantCommand(() -> drivetrain.resetPose(new Pose2d(4.414, 8.042656-7.486, new Rotation2d(0)))),
                pathBuilder.build(blueTopTrenchToTopOfBalls),
                new ParallelCommandGroup(
                        intakePivot.deploy().until(() -> intakePivot.getIntakeState() == PivotStates.Deployed),
                        new InstantCommand(() -> intakeRoller.setRollerSpeed(1)),
                        pathBuilder.build(blueTopBallsToBottomBalls)),
                new InstantCommand(() -> intakeRoller.setRollerSpeed(0)),
                // new InstantCommand(() -> intake.setRollerSpeed(0)),
                pathBuilder.build(blueBottomBallsToTopNeutralTrench),
                pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench),
                // drivetrain.BlineToHub(targetPoseHubLeft, targetPoseHubRight, 1.90, 2.40),
                drivetrain
                        .BlineToHub(1.778, 0.1, 0.1).deadlineFor(shooter.shootFuel()).andThen(
                                new ParallelCommandGroup(
                                        feeder
                                                .feederOn(0)
                                                .until(
                                                        () -> shooter
                                                                .getShooterState()
                                                                .equals(ShooterStates.Shooting) &&
                                                                drivetrain.driveBaseState.equals(
                                                                        States.InShootingPosition))
                                                .andThen(feeder.feederOn(1).alongWith(intakePivot.agitate()))

                                )));

    }
}
