package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Timer;
import java.util.TimerTask;

public class PhaseTimer extends SubsystemBase{
    private double autoTime = 20;
    private double phaseTimes = 25;
    private double endGameTime = 25;
    private int currentPhase;

    private double[] phases = {autoTime, phaseTimes, phaseTimes, phaseTimes, phaseTimes, endGameTime};
    
    private double secondsRemaining;

    private Timer timer;

    public PhaseTimer() {
        timer = new Timer();
    }

    private void runNextPhase() {
        if (currentPhase > phases.length) {return;}

        secondsRemaining = phases[currentPhase];
        timer.scheduleAtFixedRate(new TimerTask() {
            public void run() {
                if (secondsRemaining < 0) {
                    cancel();
                    currentPhase++;
                    runNextPhase();
                    return;
                }
                SmartDashboard.putNumber("Time Left in Phase: ", secondsRemaining);
                secondsRemaining--;
            }
        }, 0, 1000);
    }

   public void runTimerMain() {
        currentPhase = 0;
        runNextPhase();
   }
}