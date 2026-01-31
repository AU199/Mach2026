package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(Constants.pivotMotorId);
    private TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId);
    private PositionVoltage deployRequest = new PositionVoltage(Constants.IntakeDeployPos).withSlot(0);
    private PositionVoltage retractRequest = new PositionVoltage(Constants.IntakeRetractPos).withSlot(0);


    public Intake() {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.pivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.pivotKD;
    }

    public Command deployIntake() {
        return startEnd(() -> {
            pivotMotor.setControl(deployRequest);
        }, () -> {
            pivotMotor.set(0);
        }).until(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeDeployPos) > 0.08);

    //     return start(() -> {
    //         pivotMotor.setControl(request);
    //     });
    }

    public Command retractIntake() {
        return startEnd(() -> {
            pivotMotor.setControl(retractRequest);
        }, () -> {
            pivotMotor.set(0);
        }).until(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeRetractPos) > 0.08);
    }

    public Command runRoller() {
        return startEnd(() -> {
            rollerMotor.set(0.7);
        }, () -> {
            rollerMotor.set(0);
        });
    }
}
