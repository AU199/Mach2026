package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase{
    private TalonFX hoodMotor = new TalonFX(Constants.hoodMotorId, "DriveBase");
    
    private final double sensorToMechanismRatio = 4 * (44/16);

    private double kp;
    private double kg;
    
    public Hood() {
        hoodMotor.setPosition(Constants.hoodHardStopAngle);
    }

    public BooleanSupplier hoodReachedPosition(double targetPosition) {
        BooleanSupplier hoodReachedPosition = () -> {
            boolean result = Math.abs(hoodMotor.getPosition().getValueAsDouble() - targetPosition) < 0.01;
            return result;
        };
        return hoodReachedPosition;
    }

    public void zeroHoodPosition() {
        hoodMotor.setPosition(Constants.hoodHardStopAngle);
    }

    public Command setHoodPosition(double targetPositionMotor) {
        return runEnd(
            () -> {
                double targetPositionMechanism = targetPositionMotor / sensorToMechanismRatio; // Mechanism rotations
                double currentPosition = hoodMotor.getPosition().getValueAsDouble() / sensorToMechanismRatio; // Mechanism rotations
                double error = targetPositionMechanism - currentPosition;
                
                SmartDashboard.putNumber("Error", error);
                SmartDashboard.putNumber("Target Position", targetPositionMechanism);

                kp = error * Constants.hoodPivotKP;
                kg = Constants.hoodPivotKG  * Math.cos(currentPosition * (2 * Math.PI));
                SmartDashboard.putNumber("Cos", currentPosition * (2 * Math.PI));

                hoodMotor.setVoltage(- kp + kg);
            },
            () -> {
                hoodMotor.setVoltage(kg);
            }
        ).until(hoodReachedPosition(targetPositionMotor));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("kG", kg);
        SmartDashboard.putNumber("kP", kp);
        SmartDashboard.putNumber("Hood Angle", hoodMotor.getPosition().getValueAsDouble());
        double currentPosition = hoodMotor.getPosition().getValueAsDouble() / sensorToMechanismRatio;
        SmartDashboard.putNumber("Current Position", currentPosition);

    }
}