package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.Engineering;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot.PivotStates;

public class IntakeRollers extends SubsystemBase {
    private TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId, "DriveBase");
    private TalonFX rollerMotor2 = new TalonFX(Constants.rollerMotor2Id, "DriveBase");

    private enum RollerStates {
        Idle,
        Intaking,
        Outtaking
    }

    public IntakeRollers() {
        rollerMotor2.setControl(new Follower(Constants.rollerMotorId, MotorAlignmentValue.Opposed));
    }

    public Command runRoller(double speed, PivotStates intakePivotState) {
        if(intakePivotState.equals(PivotStates.Stowed) || intakePivotState.equals(PivotStates.Retracted)){
            return startEnd(() -> {
                rollerMotor.set(-speed);
                SmartDashboard.putNumber("Intake roller speed", speed); 
            }, () -> {
                rollerMotor.set(0);
            });
        }else{
            return new InstantCommand(() -> System.out.println("Intake has not been deployed"));
        }
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(-speed);
        SmartDashboard.putNumber("Intake roller speed", speed); 
    }
}
