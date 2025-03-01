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

import com.revrobotics.RelativeEncoder;
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
  private static final int kLiftChannel = 5;
  private static final int kDriverController = 0;
  private static final int kOperatorController = 1;
  private static final int kButtonTurbo = Constants.ControllerConstants.RT;
  private static final int kButtonGyroAdjust = Constants.ControllerConstants.LT;
  private static final int kButtonGyroReset = Constants.ControllerConstants.BACK;
  private static final int kButtonLiftUp = Constants.ControllerConstants.Y;
  private static final int kButtonLiftDown = Constants.ControllerConstants.A;
  private final MecanumDrive m_robotDrive;
  private final Joystick m_driver;
  private final Joystick m_operator;
  private AHRS gyro;
  private ShuffleboardTab tab;
  private GenericEntry val;
  private GenericEntry liftVal;
  private double gyroAdjust;
  private static RelativeEncoder liftEncoder;
  private static SparkMax lift;
  public Robot() {
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    lift = new SparkMax(kLiftChannel, MotorType.kBrushless);
    liftEncoder = lift.getEncoder();
    gyro = new AHRS(NavXComType.kMXP_SPI);
    m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, rearRight::set,frontRight::set);
    m_driver = new Joystick(kDriverController);
    m_operator = new Joystick(kOperatorController);
    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);
    tab = Shuffleboard.getTab("Custom");
    tab.add(gyro);
    val = tab.add("Apparent angle measure", 0).getEntry();
    liftVal = tab.add("Lift position", 0).getEntry();
  }

  public void teleopInit() {
    gyro.reset();
    gyroAdjust = 0;
  }
  public void teleopPeriodic() {
    if(m_driver.getRawButton(kButtonGyroReset)) {
      gyro.reset();
      gyroAdjust = 0;
    }
    double sideways = m_driver.getRawAxis(0)/5;
    double forwards = -m_driver.getRawAxis(1)/5;
    double rotation = m_driver.getRawAxis(2)/5;
    double gyroMeasure = gyro.getAngle();
    val.setDouble(gyroMeasure);
    liftVal.setDouble(liftEncoder.getPosition());
    gyroMeasure = -gyroMeasure * DEGREES_TO_RADIANS;
    if(!m_driver.getRawButton(kButtonTurbo)) {
      sideways /= 2;
      forwards /= 2;
      rotation /= 2;
    }
    if(m_driver.getRawButton(kButtonGyroAdjust)) {
      gyroAdjust += rotation / 100;
      rotation = 0;
    }
    // lift.set((m_operator.getRawButton(kButtonLiftUp) ? 1.0 : 0.0) * StrictMath.exp(-liftEncoder.getPosition() / 20) - (m_operator.getRawButton(kButtonLiftDown) ? 1.0 : 0.0) * StrictMath.exp(liftEncoder.getPosition() / 20));
    double target = 50 * (m_operator.getRawButton(kButtonLiftUp) ? 1.0 : 0.0) + 25 * (m_operator.getRawButton(kButtonLiftDown) ? 1.0 : 0.0);
    lift.set(StrictMath.exp(-(liftEncoder.getPosition() - target) / 20) - StrictMath.exp((liftEncoder.getPosition() - target) / 20));
    gyroMeasure += gyroAdjust;
    double gyroAdjustedSideways = sideways * StrictMath.cos(gyroMeasure) + forwards * StrictMath.sin(gyroMeasure);
    double gyroAdjustedForwards = forwards * StrictMath.cos(gyroMeasure) - sideways * StrictMath.sin(gyroMeasure);
    m_robotDrive.driveCartesian(-rotation, -gyroAdjustedSideways, -gyroAdjustedForwards);
  }
}