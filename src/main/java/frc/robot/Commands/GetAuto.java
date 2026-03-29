package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GetAuto {
    private final PIDController pidControllerT = new PIDController(2.3, 0, 0.2);
    private final PIDController pidControllerR = new PIDController(4, 0, 0);
    private final PIDController pidControllerCT = new PIDController(2, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public Command blueTopCollectBalls(CommandSwerveDrivetrain drivetrain) {
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
        Path blueTopBallsToBottomBalls = new Path("BlueTopBallsToBottomBalls");
        Path blueBottomBallsToTopNeutralTrench = new Path("BlueBottomBallsToTopNeutralTrench");
        Path blueTopNeutralTrenchToTopBlueTrench = new Path("BlueTopNeutralTrenchToTopBlueTrench"); 

        return Commands.sequence(pathBuilder.build(blueTopTrenchToTopOfBalls),
        pathBuilder.build(blueTopBallsToBottomBalls),
        pathBuilder.build(blueBottomBallsToTopNeutralTrench),
        pathBuilder.build(blueTopNeutralTrenchToTopBlueTrench)
        );
    }
}
