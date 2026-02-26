package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase{
    TalonFX kickerMotor = new TalonFX(Constants.kickerMotorId, "DriveBase");

    public Feeder() {}

    public Command feederOn(double speed) {
        return startEnd(() -> {
            kickerMotor.set(speed);
        }, () -> {
            kickerMotor.set(0);
        });
    }
}
