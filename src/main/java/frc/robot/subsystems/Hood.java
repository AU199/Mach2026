package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase{
    private TalonFX hoodMotor = new TalonFX(Constants.hoodMotorId, "DriveBase");
    
    private double sensorToMechanismRatio = 4 * 44/16;

    private double feedforwardOutput;
    private double feedbackOutput;

    private double kp;
    private double kg;    
    
    public Hood() {
        hoodMotor.setPosition(0);
    }

    public BooleanSupplier hoodReachedPosition(double targetPosition) {
        BooleanSupplier hoodReachedPosition = () -> {
            boolean result = Math.abs(hoodMotor.getPosition().getValueAsDouble() - targetPosition) < 0.08;
            return result;
        };
        return hoodReachedPosition;
    }

    public void zeroHoodPosition() {
        hoodMotor.setPosition(0);
    }

    public Command setHoodPosition(double targetPosition) {
        return runEnd(
            () -> {
                double currentPosition = hoodMotor.getPosition().getValueAsDouble() * sensorToMechanismRatio;
                double error = targetPosition - currentPosition;
                
                kp = error * Constants.hoodPivotKP;
                kg = Constants.hoodPivotKG  * Math.cos(currentPosition);

                hoodMotor.setVoltage(kp + kg);
            },
            () -> {
                hoodMotor.set(kg);
            }
        ).until(hoodReachedPosition(targetPosition));
    }
}
