package frc.robot;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Master {
    RobotContainer robotContainer;
    Intake intake;
    Shooter shooter;
    Feeder feeder;
    Hood hood;
    
    public Master(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        this.intake = robotContainer.getIntake();
        this.feeder = robotContainer.getFeeder();
        this.shooter = robotContainer.getShooter();
        this.hood = robotContainer.getHood();
    }

    


}
