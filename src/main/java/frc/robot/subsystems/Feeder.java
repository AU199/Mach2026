package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot.PivotStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Feeder extends SubsystemBase {
    
    public enum FeederStates {
        Idle,
        Feeding
    }

    TalonFX feederMotor = new TalonFX(Constants.feederMotorId, "DriveBase");
    TalonFX kickerMotor = new TalonFX(Constants.kickerMotorId, "DriveBase");
    private FeederStates feederState = FeederStates.Idle;

    public Feeder() {

    }

    public Command feederOn(double speed, ShooterStates shooterState, PivotStates intakePivotState) {
        if(!shooterState.equals(ShooterStates.SpinningUp)|| !intakePivotState.equals(PivotStates.Stowed) || !intakePivotState.equals(PivotStates.Retracted)){
            return startEnd(() -> {
                // feederMotor.set(-0.5);
                feederState = FeederStates.Feeding;
                kickerMotor.set(-speed);
                
            }, () -> {
                // feederMotor.set(0);
                kickerMotor.set(0);
            });}
        else{
            return new InstantCommand(()-> System.out.println("Fuh naw, currentShooter State: "+shooterState.toString()+" current Intake State: "+intakePivotState.toString()+" one of which is leading to the feeder not running"));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Motor", kickerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Kicker Motor Stator Current", kickerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Kicker Motor Supply Current", kickerMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void setFeederState(FeederStates newState) {
        feederState = newState;
    }
}