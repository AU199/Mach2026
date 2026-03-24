package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase{
    TalonFX feederMotor = new TalonFX(Constants.feederMotorId, "DriveBase");
    TalonFX kickerMotor = new TalonFX(Constants.kickerMotorId, "DriveBase");
    private Intake intakeSubsystemIntake;
    public Feeder(Intake intakeSubsystem) {
        this.intakeSubsystemIntake = intakeSubsystem;
    }

    public Command feederOn(double speed) {
        if(intakeSubsystemIntake.getIntakeMovementSupplier().getAsBoolean()){
            return startEnd(() -> {
                // feederMotor.set(-0.5);
                kickerMotor.set(-speed);
            }, () -> {
                // feederMotor.set(0);
                kickerMotor.set(0);
            });
        }
        else{
            return startEnd(() ->{
                System.out.println("Intake hasn't been deployed");
            }, () -> {});
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Motor", kickerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Kicker Motor Stator Current", kickerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Kicker Motor Supply Current", kickerMotor.getSupplyCurrent().getValueAsDouble());
    }
}