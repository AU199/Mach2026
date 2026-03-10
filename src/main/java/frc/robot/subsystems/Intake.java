package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import javax.naming.spi.DirStateFactory.Result;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

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
        pivotConfig.kP = Constants.intakePivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.intakePivotKD;
    }

    public Command setIntakePosition(double targetPosition, double KP, double maxSpeed) {
        // return startEnd(() -> {
        //     pivotMotor.setControl(deployRequest.withPosition(25));
        // }, () -> {
        //     pivotMotor.set(0);
           
        // }).until(isIntakeDeployed);

        // return runEnd(
        //     () -> {
        //         double error = targetPosition - pivotMotor.getPosition().getValueAsDouble();
        //         double output = Math.min(error * KP, maxSpeed);
        //         pivotMotor.set(output);
        //     },
        //     () -> {
        //         pivotMotor.set(0);
        //     }
        // ).until(() -> {boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPosition) < 0.08; return result;});

        // return start(() -> {
        //     pivotMotor.setControl(request);
        // });

        return startEnd(() -> {
            // in init function
            var talonFXConfigs = new TalonFXConfiguration();

            // set slot 0 gains
            var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
            slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
            slot0Configs.withGravityType(GravityTypeValue.valueOf(GravityTypeValue.Arm_Cosine.value)); // Use cosine gravity compensation
            slot0Configs.withGravityArmPositionOffset(Constants.intakeHardStopAngle/ (2.0*Math.PI)); // Set the position of the hard stop as the zero point for gravity compensation
            slot0Configs.kP = Constants.intakePivotKP; // A position error of 2.5 rotations results in 12 V output
            slot0Configs.kI = Constants.intakePivotKI; // no output for integrated error
            slot0Configs.kD = Constants.intakePivotKD; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
            var motionMagicConfigs = talonFXConfigs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
            motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
            motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

            pivotMotor.getConfigurator().apply(talonFXConfigs);
        }, () -> {
            pivotMotor.set(0);
        });
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
