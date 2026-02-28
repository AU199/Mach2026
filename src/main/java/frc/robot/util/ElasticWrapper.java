package frc.robot.util;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;

import frc.robot.util.Elastic.*;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElasticWrapper {

    Elastic.Notification notification = new Notification();
    double shiftTimer = 0;
    double matchTime;
    String gameData;
    boolean RedActive = false;
    public ElasticWrapper(){

    }

    public void hubTimer(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            Elastic.sendNotification(notification
            .withLevel(NotificationLevel.ERROR)
            .withTitle("Alliance")
            .withDescription("NO ALLIANCE HAS BEEN SEEN, IF AT COMP MAKE SURE WITH FTA")
            .withDisplaySeconds(5));
            return;
        }
        if(DriverStation.isAutonomousEnabled()){
            SmartDashboard.putBoolean("Hub Active", true);
            return;
        }
        if(!DriverStation.isTeleopEnabled()){
            Elastic.sendNotification(notification
            .withLevel(NotificationLevel.ERROR)
            .withTitle("Tele-op")
            .withDescription("Tele-op wasn't enabled after autonomus, IF AT COMP MAKE SURE WITH FTA")
            .withDisplaySeconds(5));
            SmartDashboard.putBoolean("Hub Active", false);
            return;
        }
        matchTime = DriverStation.getMatchTime();
        gameData = DriverStation.getGameSpecificMessage();

        if(gameData.isEmpty()){
            Elastic.sendNotification(notification
            .withLevel(NotificationLevel.ERROR)
            .withTitle("FMS")
            .withDescription("Game Specific Message didn't return anything, if this is not in the early game BADDDDD, IF AT COMP MAKE SURE WITH FTA")
            .withDisplaySeconds(5));
            SmartDashboard.putBoolean("Hub Active", true);
            return;   
        }
        switch(gameData.charAt(0)){
            case 'R' -> RedActive = true;
            case 'B' -> RedActive = false;
            default ->{
                return;
            }
        }


        if (matchTime > 130){
            Elastic.sendNotification(notification
            .withLevel(NotificationLevel.INFO)
            .withTitle("Current Shift")
            .withDescription("Transition Shift")
            .withDisplaySeconds(4)
            );
            SmartDashboard.putBoolean("Hub Active", true);
        }
        
    }

}
