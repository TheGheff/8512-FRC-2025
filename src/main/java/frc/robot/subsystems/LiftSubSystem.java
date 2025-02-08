// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
public class LiftSubSystem extends SubsystemBase {
  private final SparkMax m_liftMotorSparkMax = new SparkMax(DriveConstants.kLiftChannel, MotorType.kBrushless);
  /** Creates a new ExampleSubsystem. */
  public LiftSubSystem() {
    // this.m_liftMotorSparkMax = liftMotorSparkMax;

    //setDefaultCommand(
     // runOnce(
//()->{m_liftMotorSparkMax.disable();})

   // );
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command raise() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_liftMotorSparkMax.set(0.1);
        });
  }
  public Command lower() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_liftMotorSparkMax.set(-0.1);
        });
  }
  public Command stop() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_liftMotorSparkMax.set(0.0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
