// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    
  public static final int pivotMotorId = 0;
  public static final int rollerMotorId = 0;
  public static final int feederMotorId = 0;
  public static final int frontShooter1Id = 0;
  public static final int frontShooter2Id = 0;
  public static final int backShooterId = 0;
  public static final int hoodMotorId = 0;

  public static final double pivotKP = 0;
  public static final double pivotKI = 0;
  public static final double pivotKD = 0;

  public static final double IntakeDeployPos = 0;
  public static final double IntakeRetractPos = 0;
}