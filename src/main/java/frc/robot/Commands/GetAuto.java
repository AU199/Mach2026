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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;

public class GetAuto {
    private final PIDController pidControllerT = new PIDController(2.3, 0, 0.2);
    private final PIDController pidControllerR = new PIDController(4, 0, 0);
    private final PIDController pidControllerCT = new PIDController(2, 0, 0);
    private final double x = 2.362, y = 5.205;
    private final Pose2d targetPoseHubLeft = new Pose2d(x, y,
            new Rotation2d(Constants.blueHubPose.getX() - x,
                    Constants.blueHubPose.getY() - y).plus(new Rotation2d(Math.PI)));
    private final Pose2d targetPoseHubRight = new Pose2d(x, y,
            new Rotation2d(Constants.blueHubPose.getX() - x,
                    Constants.blueHubPose.getY() - (8 - y)).plus(new Rotation2d((3 * Math.PI) / 2)));
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public Command blueTopCollectBalls(CommandSwerveDrivetrain drivetrain, IntakePivot intake, Shooter shooter, Hood hood,
            Feeder feeder) {
        pidControllerR.setTolerance(0.75);
        pidControllerT.setTolerance(0.75);
        pidControllerCT.setTolerance(0.75);

        FollowPath.Builder pathBuilder = new FollowPath.Builder(
                drivetrain,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                (speeds) -> drivetrain.setControl(autoRequest.withSpeeds(speeds)),
                pidControllerT, pidControllerR, pidControllerCT);

        Path blueTopTrenchToTopOfBalls = new Path("BlueTopTrenchToTopOfBalls");
        blueTopTrenchToTopOfBalls.mirror();
        Path blueTopBallsToBottomBalls = new Path("BlueTopBallsToBottomBalls");
        blueTopBallsToBottomBalls.mirror();
        Path blueBottomBallsToTopNeutralTrench = new Path("BlueBottomBallsToTopNeutralTrench");
        blueBottomBallsToTopNeutralTrench.mirror();
        Path blueTopNeutralTrenchToTopBlueTrench = new Path("BlueTopNeutralTrenchToTopBlueTrench");
        blueTopNeutralTrenchToTopBlueTrench.mirror();

        return Commands.sequence(
                pathBuilder.build(blueTopTrenchToTopOfBalls),
                intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5),
                new InstantCommand(() -> intake.setRollerSpeed(1)),
                pathBuilder.build(blueTopBallsToBottomBalls),
                new InstantCommand(() -> intake.setRollerSpeed(0)),
                pathBuilder.build(blueBottomBallsToTopNeutralTrench),
                pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench),
                //drivetrain.BlineToHub(targetPoseHubLeft, targetPoseHubRight, 1.90, 2.40),
                new ParallelCommandGroup(
                        hood.setHoodPosition(0.1),
                        shooter.shooterOn(50),
                        new SequentialCommandGroup(
                                new WaitCommand(5),
                                feeder.feederOn(1))));
    }
}
