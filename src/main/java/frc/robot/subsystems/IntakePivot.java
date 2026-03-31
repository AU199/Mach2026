package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    public enum PivotStates {
        Stowed,
        Retracted,
        Retracting,
        Deployed,
        Deploying,
        Depot,
        Depoting,
        Agitating,
    }

    private TalonFX pivotMotor = new TalonFX(Constants.pivotMotorId, "DriveBase");
    private PivotStates intakePivotState = PivotStates.Stowed;

    BooleanSupplier isIntakeMoved = () -> {
        boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - 0) > 0.08;
        return result;
    };
    // BooleanSupplier isIntakeRetracted = () -> {
    // boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() -
    // Constants.IntakeRetractPos) < 0.08;
    // return result;
    // };

    public IntakePivot() {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.intakePivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.intakePivotKD;
    }

    // public Command setIntakePosition(double targetPosition, double KP, double
    // maxSpeed) {
    // return runEnd(
    // () -> {
    // SmartDashboard.putNumber("Intake target position", targetPosition);
    // double error = targetPosition - pivotMotor.getPosition().getValueAsDouble();
    // double output = Math.min(error * KP, maxSpeed);
    // pivotMotor.set(output);
    // },
    // () -> {
    // pivotMotor.set(0);
    // }
    // ).until(() -> {boolean result =
    // Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPosition) <
    // 0.08; return result;});

    // }

    public Command deploy(double KP, double maxSpeed) {
        if (!intakePivotState.equals(PivotStates.Deployed)) {
            return runEnd(
                    () -> {
                        SmartDashboard.putNumber("Intake target position", Constants.IntakeDeployPos);
                        double error = Constants.IntakeDeployPos - pivotMotor.getPosition().getValueAsDouble();
                        double output = Math.min(error * KP, maxSpeed);
                        pivotMotor.set(output);
                        intakePivotState = PivotStates.Deploying;
                    },
                    () -> {
                        pivotMotor.set(0);
                    }

            )
                    .until(() -> {
                        boolean result = Math
                                .abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeDeployPos) < 0.08;
                        return result;
                    })
                    .andThen(new InstantCommand(() -> intakePivotState = PivotStates.Deployed));
        } else {
            return new InstantCommand(
                    () -> System.out.println("INTAKE ALREADY DEPLOYED"));
        }

    }

    public Command retract(double KP, double maxSpeed) {
        if (!intakePivotState.equals(PivotStates.Retracted) || !intakePivotState.equals(PivotStates.Stowed)) {
            return runEnd(
                () -> {
                    SmartDashboard.putNumber("Intake target position", Constants.IntakeRetractPos);
                    double error = Constants.IntakeRetractPos - pivotMotor.getPosition().getValueAsDouble();
                    double output = Math.min(error * KP, maxSpeed);
                    pivotMotor.set(output);
                    intakePivotState = PivotStates.Retracting;
                },
                () -> {
                    pivotMotor.set(0);
                }
            ).until(() -> {
                boolean result = Math
                    .abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeRetractPos) < 0.08;
                return result;
            }).andThen(new InstantCommand(() -> intakePivotState = PivotStates.Retracted));
        } else {
            return new InstantCommand(
                () -> System.out.println("INTAKE WAS NEVER DEPLOYED"));
        }

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
                });
    }

    public PivotStates getIntakeState() {
        return intakePivotState;
    }
}
