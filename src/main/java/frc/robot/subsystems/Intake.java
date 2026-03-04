package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import javax.naming.spi.DirStateFactory.Result;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(Constants.pivotMotorId, "DriveBase");
    private TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId, "DriveBase");
    private PositionVoltage deployRequest = new PositionVoltage(Constants.IntakeDeployPos).withSlot(0);
    private PositionVoltage retractRequest = new PositionVoltage(Constants.IntakeRetractPos).withSlot(0);

    // BooleanSupplier isIntakeDeployed = () -> {
    //     boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeDeployPos) < 0.08;
    //   return result;
    // };
    // BooleanSupplier isIntakeRetracted = () -> {
    //     boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeRetractPos) < 0.08;
    //   return result;
    // };

    public Intake() {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.intakePivotKD;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.intakePivotKD;
    }

    public Command setIntakePosition(double targetPosition, double KP, double maxSpeed) {
        // return startEnd(() -> {
        //     pivotMotor.setControl(deployRequest.withPosition(25));
        // }, () -> {
        //     pivotMotor.set(0);
           
        // }).until(isIntakeDeployed);
        return runEnd(
            () -> {
                double error = targetPosition - pivotMotor.getPosition().getValueAsDouble();
                double output = Math.min(error * KP, maxSpeed);
                pivotMotor.set(output);
            },
            () -> {
                pivotMotor.set(0);
            }
        ).until(() -> {boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPosition) < 0.08; return result;});

        // return start(() -> {
        //     pivotMotor.setControl(request);
        // });
    }

    public Command runRoller(double speed) {
        return startEnd(() -> {
            rollerMotor.set(speed);
        }, () -> {
            rollerMotor.set(0);
        });
    }

    public Command runPivotSetSpeed(double speed) {
        return startEnd(() -> {
            pivotMotor.set(speed);
        }, () -> {
            pivotMotor.set(0);
        });
    }

    public Command zeroPivotEncoder() {
        return new InstantCommand(
            () -> {
                pivotMotor.setPosition(0);
            }
        );
    }
}
