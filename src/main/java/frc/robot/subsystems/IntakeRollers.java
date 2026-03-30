package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {
    private TalonFX rollerMotor = new TalonFX(Constants.rollerMotorId, "DriveBase");
    private TalonFX rollerMotor2 = new TalonFX(Constants.rollerMotor2Id, "DriveBase");

    private enum States {
        Idle,
        Intaking,
        Outtaking
    }

    public IntakeRollers() {
        rollerMotor2.setControl(new Follower(Constants.rollerMotorId, MotorAlignmentValue.Opposed));
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
}
