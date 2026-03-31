package frc.robot.Commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;

public class BusterAuto extends SequentialCommandGroup {
    private RobotContainer robotContainer;
    private SendableChooser<String> chooserAuto;
    private double angle;
    private CommandSwerveDrivetrain drivebase;

    public BusterAuto(RobotContainer robotContainer, SendableChooser<String> chooserAuto,
            CommandSwerveDrivetrain drivebase, IntakePivot intakePivot, IntakeRollers intakeRollers, Shooter shooter, Hood hood,Feeder feeder) {
        this.robotContainer = robotContainer;
        this.chooserAuto = chooserAuto;
        this.drivebase = drivebase;
        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
        SmartDashboard.putBoolean("isRed", isRed);

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
                addCommands(new GetAuto().blueTopCollectBalls(drivebase, intakePivot, intakeRollers, shooter, hood, feeder));
                break;
//             case "right":
//                 // PathPlannerPath path = PathPlannerPath.fromPathFile("1 coral");

//                 if (isRed) {
//                     angle = 0;
//                 } else {
//                     angle = Math.PI;
//                 }

//                 addCommands(
//                         new InstantCommand(() -> drivebase.resetRotation(new Rotation2d(angle))),
//                         new PathPlannerAuto("3 coral auto left", true));
//                 break;
//             case "left":
//                 if (isRed) {
//                     angle = 0;
//                 } else {
//                     angle = Math.PI;
//                 }
//                 addCommands(
//                         new InstantCommand(() -> drivebase.resetRotation(new Rotation2d(angle))),
//                         new PathPlannerAuto("3 coral auto left"));
//                 break;
//             case "midleft":
//                 if (isRed) {
//                     angle = 0;
//                 } else {
//                     angle = Math.PI;
//                 }
//                 addCommands(
//                         new InstantCommand(() -> drivebase.resetRotation(new Rotation2d(angle))),
//                         new PathPlannerAuto("1 coral middle left"));
//                 break;
//             case "midright":
//                 if (isRed) {
//                     angle = 0;
//                 } else {
//                     angle = Math.PI;
//                 }
//                 addCommands(
//                         new InstantCommand(() -> drivebase.resetRotation(new Rotation2d(angle))),
//                         new PathPlannerAuto("1 coral middle left", true));
//                 break;

        }
    }
}