// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.LiftSubSystem;
public class Robot extends TimedRobot {
  private static final int kJoystickChannel = 0;
  SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftChannel, MotorType.kBrushless);
  SparkMax rearLeft = new SparkMax(DriveConstants.kRearLeftChannel, MotorType.kBrushless);
  SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightChannel, MotorType.kBrushless);
  SparkMax rearRight = new SparkMax(DriveConstants.kRearRightChannel, MotorType.kBrushless);
  // SparkMax lift = new SparkMax(DriveConstants.kLiftChannel, MotorType.kBrushless);
  AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  private final MecanumDrive m_robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, rearRight::set,frontRight::set);
  private final Joystick m_stick = new Joystick(kJoystickChannel);
  private final JoystickButton kButtonTurbo = new JoystickButton(m_stick, ControllerConstants.RT);
  private final JoystickButton kButtonRaise = new JoystickButton(m_stick, ControllerConstants.RB);
  private final JoystickButton kButtonLower = new JoystickButton(m_stick, ControllerConstants.LB);
  private final LiftSubSystem m_lift = new LiftSubSystem();

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);
    SendableRegistry.addChild(m_gyro, m_gyro.getRotation2d());
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    kButtonRaise.onTrue(m_lift.raise());
    kButtonRaise.onFalse(m_lift.stop());
    kButtonLower.onTrue(m_lift.lower());
    kButtonLower.onFalse(m_lift.stop());

    m_gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    if(kButtonTurbo.getAsBoolean()) {
      m_robotDrive.driveCartesian(-m_stick.getRawAxis(2)/5, -m_stick.getRawAxis(0)/5, m_stick.getRawAxis(1)/5, m_gyro.getRotation2d());
    } else {
      m_robotDrive.driveCartesian(-m_stick.getRawAxis(2)/10, -m_stick.getRawAxis(0)/10, m_stick.getRawAxis(1)/10, m_gyro.getRotation2d());
    }
  }
}