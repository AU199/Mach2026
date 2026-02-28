package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase{
    TalonFX feederMotor = new TalonFX(Constants.feederMotorId, "DriveBase");
    TalonFX kickerMotor = new TalonFX(Constants.kickerMotorId, "DriveBase");
 
    public Feeder() {}

    public Command feederOn(double speed) {
        return startEnd(() -> {
            //feederMotor.set(-speed);
            kickerMotor.set(-speed);
        }, () -> {
            feederMotor.set(0);
            kickerMotor.set(0);
        });
    }
}
