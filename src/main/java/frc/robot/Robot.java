// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.lang.StrictMath;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Robot extends TimedRobot {
  /*import java.lang.StrictMath.DEGREES_TO_RADIANS;*/
  private static final double DEGREES_TO_RADIANS = 0.017453292519943295;
  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 4;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  private static final int kJoystickChannel = 0;
  private static final int kButtonTurbo = Constants.ControllerConstants.RT;
  private static final int kButtonGyroAdjust = Constants.ControllerConstants.LT;
  private final MecanumDrive m_robotDrive;
  private final Joystick m_stick;
  private AHRS gyro;
  private ShuffleboardTab tab;
  private GenericEntry val;
  private double gyroAdjust;
  private double gyroAdjustTime;
  public Robot() {
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    gyro = new AHRS(NavXComType.kMXP_SPI);
    m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, rearRight::set,frontRight::set);
    m_stick = new Joystick(kJoystickChannel);
    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);
    tab = Shuffleboard.getTab("Custom");
    tab.add(gyro);
    val = tab.add("Apparent angle measure", 0).getEntry();
  }

  public void teleopInit() {
    gyro.reset();
    gyroAdjust = 0;
    gyroAdjustTime = 0;
  }
  public void teleopPeriodic() {
    double sideways = m_stick.getRawAxis(0)/5;
    double forwards = -m_stick.getRawAxis(1)/5;
    double rotation = m_stick.getRawAxis(2)/5;
    double gyroMeasure = gyro.getAngle();
    val.setDouble(gyroMeasure);
    gyroMeasure = -gyroMeasure * DEGREES_TO_RADIANS;
    if(!m_stick.getRawButton(kButtonTurbo)) {
      sideways = sideways/2;
      forwards = forwards/2;
      rotation = rotation/2;
    }
    if(m_stick.getRawButton(kButtonGyroAdjust)) {
      rotation = 0;
      gyroAdjustTime += rotation / 100;
    } else {
      gyroAdjustTime = 0;
    }
    gyroAdjust += gyroAdjustTime;
    gyro.setAngleAdjustment(gyroAdjust);
    double gyroAdjustedSideways = sideways * StrictMath.cos(gyroMeasure) + forwards * StrictMath.sin(gyroMeasure);
    double gyroAdjustedForwards = forwards * StrictMath.cos(gyroMeasure) - sideways * StrictMath.sin(gyroMeasure);
    m_robotDrive.driveCartesian(-rotation, -gyroAdjustedSideways, -gyroAdjustedForwards);
  }
}