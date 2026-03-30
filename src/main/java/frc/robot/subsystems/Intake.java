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
import frc.robot.Constants.IntakeStates;

public class Intake extends SubsystemBase {

    


    public IntakeStates currentState = IntakeStates.RETRACTED;
    private TalonFX pivotMotor = new TalonFX(Constants.pivotMotorId, "DriveBase");
    private TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId, "DriveBase");
    private TalonFX rollerMotor2 = new TalonFX(Constants.rollerMotor2Id, "DriveBase");
    BooleanSupplier isIntakeMoved = () -> {
        boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - 0) > 0.08;
      return result;
    };
    // BooleanSupplier isIntakeRetracted = () -> {
    //     boolean result = Math.abs(pivotMotor.getPosition().getValueAsDouble() - Constants.IntakeRetractPos) < 0.08;
    //   return result;
    // };

    public Intake() {
        Slot0Configs pivotConfig = new Slot0Configs();
        pivotConfig.kP = Constants.intakePivotKP;
        pivotConfig.kI = 0;
        pivotConfig.kD = Constants.intakePivotKD;

        rollerMotor2.setControl(new Follower(Constants.rollerMotorId, MotorAlignmentValue.Opposed));
    }

    public void setIntakePosition(double targetPosition, double KP, double maxSpeed) {
        
        SmartDashboard.putNumber("Intake target position", targetPosition);
        double error = targetPosition - pivotMotor.getPosition().getValueAsDouble();
        double output = Math.min(error * KP, maxSpeed);
        pivotMotor.set(output);
    }

    public Command runRoller(double speed) {
        // if(isIntakeMoved.getAsBoolean()){
            return startEnd(() -> {
                rollerMotor.set(-speed);
                SmartDashboard.putNumber("Intake roller speed", speed); 
            }, () -> {
                rollerMotor.set(0);
            });
        // } else {
            //This is a fuction that doesn't do anything
        //     return startEnd(() -> {}, 
        //     () -> {});
        // }
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(-speed);
        SmartDashboard.putNumber("Intake roller speed", speed); 
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

    public BooleanSupplier getIntakeMovementSupplier(){
        return isIntakeMoved;
    }

    public void stop() {
        pivotMotor.set(0);
        rollerMotor.set(0);
    }

    public void agitate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'agitate'");
    }
}
