// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; 
  }
  public static final double robotRadius = 12.5*0.0254;
    
  public static final int pivotMotorId = 13;
  public static final int rollerMotorId = 14;
  public static final int feederMotorId = 16;
  public static final int frontShooter1Id = 17;
  public static final int frontShooter2Id = 18;
  public static final int backShooterId = 19;
  public static final int hoodMotorId = 20;
  public static final int hangArmId = 15;

  public static final double pivotKP = 0;
  public static final double pivotKI = 0;
  public static final double pivotKD = 0;

  public static final Pose2d blueHubPose = new Pose2d(182.11*0.0254, 158.84*0.0254, new Rotation2d(0));
  public static final Pose2d redHubPose = new Pose2d(469.11*0.0254, 158.84*0.0254, new Rotation2d(0));
  public static final double hubZ = 0;

  public static final double shooterHeight = 0;
  public static final double shooterPositionX = 0;
  public static final double shooterPositionY = 0;

  public static final double ballInitialVelocityFromShooter = 8.5;

  public static final double IntakeDeployPos = 0;
  public static final double IntakeRetractPos = 0;
  public static final String Path = Paths.get("").toAbsolutePath().toString()+"src\\\\main\\\\java\\\\frc\\\\robot\\\\AprilTags\\\\2026-rebuilt-welded.json";
  public static AprilTagFieldLayout kTagLayout;
  static{ 
    try{
      kTagLayout = new AprilTagFieldLayout(Path);
  
    }catch(IOException e){
      System.out.println(e.getStackTrace());
    }
  }
  public static final Transform3d kRobotToCam = new Transform3d(20, 0,20 ,new Rotation3d(0,0,0));
}