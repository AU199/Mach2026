package frc.robot.subsystems.Managers;

import frc.robot.Constants;
import frc.robot.Constants.IntakeStates;
import frc.robot.subsystems.Intake;

public class IntakeManager {
    Intake intake;
    public IntakeManager(Intake intake){
        this.intake = intake;

    }

    public void setDesiredState(IntakeStates State){
        switch (State) {
            case DEPLOYED:
                intake.setIntakePosition(Constants.IntakeDeployPos, 0.1, 0.5);
                intake.currentState = IntakeStates.DEPLOYED;
                break;
            case RETRACTED:
                intake.setIntakePosition(Constants.IntakeRetractPos, 0.025, 0.3);
                intake.currentState = IntakeStates.RETRACTED;
                break;

            case INTAKING:
                intake.runRoller(1);
                intake.currentState = IntakeStates.INTAKING;
                break;
            
            case AGITATIGING:
                intake.agitate();
            

            case IDLE:
                intake.stop();
                intake.currentState = IntakeStates.IDLE;

            default:

                break;

        }
    }



}
