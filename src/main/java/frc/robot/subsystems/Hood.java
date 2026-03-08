package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase{
    private TalonFX hoodMotor = new TalonFX(Constants.hoodMotorId, "DriveBase");
    
    
    private final double sensorToMechanismRatio = 11; // (48/12) * (44/16)

    private double kp;
    private double kg;
    
    public Hood() {
        hoodMotor.setPosition(Constants.hoodHardStopAngle);
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.withGravityType(GravityTypeValue.valueOf(GravityTypeValue.Arm_Cosine.value)); // Use cosine gravity compensation
        slot0Configs.withGravityArmPositionOffset(Constants.hoodHardStopAngle/ (2.0*Math.PI)); // Set the position of the hard stop as the zero point for gravity compensation
        slot0Configs.kP = Constants.hoodPivotKP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = Constants.hoodPivotKI; // no output for integrated error
        slot0Configs.kD = Constants.hoodPivotKD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        hoodMotor.getConfigurator().apply(talonFXConfigs);
    }

    public BooleanSupplier hoodReachedPosition(double targetPosition) {
        BooleanSupplier hoodReachedPosition = () -> {
            boolean result = Math.abs(hoodMotor.getPosition().getValueAsDouble() - targetPosition) < 0.01;
            return result;
        };
        return hoodReachedPosition;
    }

    public void zeroHoodPosition() {
        hoodMotor.setPosition(Constants.hoodHardStopAngle); // RETUNE
    }

    public Command setHoodPosition(double targetPositionMotor) {
        return runEnd(
            () -> {
                double targetPositionMechanism = targetPositionMotor / sensorToMechanismRatio; // Mechanism rotations
                double currentPosition = hoodMotor.getPosition().getValueAsDouble() / sensorToMechanismRatio; // Mechanism rotations
                double error = targetPositionMechanism - currentPosition; // Mechanism rotations
                SmartDashboard.putNumber("Error", error);
                SmartDashboard.putNumber("Current Position", currentPosition);
                SmartDashboard.putNumber("Target Position", targetPositionMechanism);
                MotionMagicVoltage control = new MotionMagicVoltage(targetPositionMechanism); // Target position in mechanism rotations

                var controlInfo = control.getControlInfo();
                SmartDashboard.putString("Feedforward", controlInfo.get("FeedForward").toString());
                SmartDashboard.putNumber("Cos", currentPosition * (2 * Math.PI));
                
                hoodMotor.setControl(control); // Target position in mechanism rotations, feedforward in volts
            
            },
            () -> {
                hoodMotor.setVoltage(kg);
            }
        );

        // ).until(hoodReachedPosition(targetPositionMotor));
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