// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 4;
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 3;
  private static final int kJoystickChannel = 0;
  private static final int kButtonTurbo = Constants.ControllerConstants.RT;
  private final MecanumDrive m_robotDrive;
  private final Joystick m_stick;

  public Robot() {
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);
    // frontLeft.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, rearRight::set,frontRight::set);//frontRight::set, rearRight::set);
    m_stick = new Joystick(kJoystickChannel);
    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);
  }

  @Override
  public void teleopPeriodic() {
    if(m_stick.getRawButton(kButtonTurbo)) {
      m_robotDrive.driveCartesian(-m_stick.getRawAxis(2)/10, -m_stick.getRawAxis(0)/10, m_stick.getRawAxis(1)/10);
    } else {
      m_robotDrive.driveCartesian(-m_stick.getRawAxis(2)/20, -m_stick.getRawAxis(0)/20, m_stick.getRawAxis(1)/20);
    }
  }
  // @Override
  // public void teleopPeriodic() {
  //   m_robotDrive.driveCartesian(-m_stick.getRawAxis(0), -m_stick.getRawAxis(1), (m_stick.getRawAxis(5)-m_stick.getRawAxis(4)));
  // }
}