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
  public static class DriveConstants {
    //Motor Controller Assignments
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 2;
    public static final double kGearboxRatio = 1/12.75; //Ratio = 1:12.75
    public static final double kWheelRadius = 0.0762; //3in = 0.0762 meters
    public static final double kTrackWidth = 0.56; // meters
    public static final double kDriveP = 1; 
    public static final double kDriveI = 10e-7;
    public static final double kDriveD = 1;
    public static final double kDriveIz = 0; 
    public static final double kDriveFF = 0; 
    public static final double kDriveMaxOutput = 1; 
    public static final double kDriveMinOutput = -1;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // public static class ShooterConstants{
  //   public static final int kOuterWheelPort = 7;
  //   public static final int kInnerWheelPort = 6;
  //   public static final int kLoadDelay = 1; //sec
  //   public static final int kFireDelay = 2; //sec
  // }

  // public static class DropperConstants{
  //   public static final double kP = 1; 
  //   public static final double kI = 10e-7;
  //   public static final double kD = 1; 
  //   public static final double kIz = 0; 
  //   public static final double kFF = 0; 
  //   public static final double kMaxOutput = 1; 
  //   public static final double kMinOutput = -1;

  //   public static final int kDropperPort = 5;
  //   public static final double kExtendedPosition = 0.36;
  //   public static final int kRetractedPosition = 0;
  // }

  public static class RobotConstants{
    public static final double kTurnRateScaler = 0.8;
    public static final double kMaxSpeedLimit = 0.75;
    public static final double kSlowSpeedLimit = 0.5;
  }

  public static class ControllerConstants{
    public static final int X = 1;
    public static final int A = 2;
    public static final int B = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int LT = 7;
    public static final int RT = 8;
    public static final int BACK = 9;
    public static final int START = 10;
    public static final int L3 = 11;
    public static final int R3 = 12;
    public static final int L_V_Axis = 1;
    public static final int L_H_Axis = 2;
    public static final int R_V_Axis = 3;
    public static final int R_H_Axis = 4;
  }

  public static class AutonomousConstants{
    public static final String kDefaultAuto = "Default Auto";
  }
}
