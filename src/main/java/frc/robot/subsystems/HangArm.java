package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangArm  extends SubsystemBase{
    private SparkMax hangArm = new SparkMax(Constants.hangArmId, MotorType.kBrushless);
    
    public HangArm() {}

    public Command runHangArm(double speed) {
        return startEnd(() -> {
            hangArm.set(speed);
        }, () -> {
            hangArm.set(0);
        });
    }
}
