package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;

public class PhaseTimer{
    private boolean hubActive;
    private double nextTime = 130;
    
    public PhaseTimer(){
    }

    
    public boolean IsItOurTurn(Optional<Alliance> alliance, String gameData, double matchTime){
        Alliance shift2and4Team = gameData.charAt(0) == 'R'? Alliance.Red: Alliance.Blue;
        boolean areWeShift2and4Team = alliance.equals(shift2and4Team)? true: false;
        if(matchTime > 130){
            return true;
        }
        if (matchTime>105){
            return !areWeShift2and4Team;
        }
        if (matchTime > 80){
            return areWeShift2and4Team;
        }
        if (matchTime > 55){
            return !areWeShift2and4Team;
        }
        if(matchTime >30){
            return areWeShift2and4Team;
        }else{
            return true;
        }
    }

    public void updateTimings(double currentMatchTime, boolean ourTurn){
        int differenceInTime = (int) (currentMatchTime - nextTime);
        if((differenceInTime < 0 )&& (nextTime>30)){
            nextTime -= 25;
        }else if(differenceInTime < 0){
            nextTime -= 30;
        }
        else{
            if(ourTurn){
                SmartDashboard.putNumber("Hub Active Time", differenceInTime);
                SmartDashboard.putNumber("Until next Hub Active Period", 0);
            }else{
                SmartDashboard.putNumber("Hub Active Time", 0);
                SmartDashboard.putNumber("Until next Hub Active Period", differenceInTime);
                
            }
            
        }
    }
    
    public void gameDataSender(){
        var alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()){
            SmartDashboard.putString("Alliance","");
            return;
        }else{
            SmartDashboard.putString("Alliance", alliance.equals(Alliance.Red)? "Red":"Blue");
        }

        if(DriverStation.isAutonomous()){
            hubActive = true;
            SmartDashboard.putBoolean("Hub Active?", hubActive);
            return;
        }

        if(!DriverStation.isTeleopEnabled()){
            System.out.println("No hub, very bad");
            hubActive = false;
            SmartDashboard.putBoolean("Hub Active", hubActive);
            return;
        }

        double matchTime = DriverStation.getMatchTime();

        String gameData = DriverStation.getGameSpecificMessage();

        //This is set to true since it is possible that it is still early in tele-op.

        if (gameData.isEmpty()){
            hubActive = true;
            SmartDashboard.putBoolean("Hub Active?", hubActive);
            return;

        }
        // Passing in alliance over here should be fine since it has been check whether or not it was empty before this.
        hubActive = IsItOurTurn(alliance, gameData, matchTime);
        SmartDashboard.putBoolean("Hub Active?", hubActive);
        updateTimings(matchTime, hubActive);
        




    }


}