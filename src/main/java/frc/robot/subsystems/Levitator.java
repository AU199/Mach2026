package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Levitator extends SubsystemBase {

    private final TalonFX levitatorMotor;
    private final PositionVoltage positionVoltage;
    private Slot0Configs levitatorPIDConstants;

    public static final double LEVITATOR_LIFT_TIME = 2.5;
    public static final double LEVITATOR_RETRACT_TIME = 2.5;
    public static final double LEVITATOR_LIFT_POSITION = -10.0;
    public static final double LEVITATOR_RETRACT_POSITION = -100.0;

    public Levitator() {
        levitatorMotor = new TalonFX(Constants.levitatorMotorId, "DriveBase");
        positionVoltage = new PositionVoltage(0).withSlot(0);

        levitatorPIDConstants = new Slot0Configs();
        levitatorPIDConstants.kP = Constants.levitatorKP;
        levitatorPIDConstants.kI = Constants.levitatorKI;
        levitatorPIDConstants.kD = Constants.levitatorKD;
    }

    public Command lift() {
        return startEnd(
            () -> {
                levitatorMotor.setControl(
                    positionVoltage.withPosition(LEVITATOR_LIFT_POSITION)
                );
                System.out.println("LEVITATOR: Lifting");
            },
            () -> {
                System.out.println("LEVITATOR: Lifted");
                levitatorMotor.set(0);
            }
        )
            .until(this::done_lifting)
            .withTimeout(LEVITATOR_LIFT_TIME);
    }

    public Command retract() {
        return startEnd(
            () -> {
                levitatorMotor.setControl(
                    positionVoltage.withPosition(LEVITATOR_RETRACT_POSITION)
                );
                System.out.println("LEVITATOR: Retracting");
            },
            () -> {
                System.out.println("LEVITATOR: Retracted");
                levitatorMotor.set(0);
            }
        )
            .until(this::done_retracting)
            .withTimeout(LEVITATOR_RETRACT_TIME);
    }

    public Command runLevitator(double speed) {
        return startEnd(
            () -> {
                levitatorMotor.set(speed);
            },
            () -> {
                levitatorMotor.set(0);
            }
        );
    }

    private boolean done_lifting() {
        return (
            levitatorMotor.getPosition().getValueAsDouble() >
            LEVITATOR_LIFT_POSITION
        );
    }

    private boolean done_retracting() {
        return (
            levitatorMotor.getPosition().getValueAsDouble() <
            LEVITATOR_RETRACT_POSITION
        );
    }

    @Override
    public void periodic() {}
}
