package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANDriveSubsystem  extends SubsystemBase{
    private final SparkMax frontLeft;
    private final SparkMax rearLeft;
    private final SparkMax frontRight;
    private final SparkMax rearRight;

    private final AHRS m_gyro;

    private final MecanumDrive robotDrive;

    public CANDriveSubsystem() {


    frontLeft = new SparkMax(DriveConstants.kFrontLeftChannel, MotorType.kBrushless);
    rearLeft = new SparkMax(DriveConstants.kRearLeftChannel, MotorType.kBrushless);
    frontRight = new SparkMax(DriveConstants.kFrontRightChannel, MotorType.kBrushless);
    rearRight = new SparkMax(DriveConstants.kRearRightChannel, MotorType.kBrushless);
  
    robotDrive = new MecanumDrive(frontLeft, rearLeft, rearRight,frontRight);

    SendableRegistry.addChild(robotDrive, frontLeft);
    SendableRegistry.addChild(robotDrive, rearLeft);
    SendableRegistry.addChild(robotDrive, frontRight);
    SendableRegistry.addChild(robotDrive, rearRight);

    
    m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(12);
    // config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // frontRight.setInverted(true);
    // rearRight.setInverted(true);

    }

   @Override
  public void periodic() {
  }
  //gyro interface
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // Command to drive the robot with joystick inputs
  public Command drive(
    CANDriveSubsystem driveSubsystem,  Double yAxix, Double xAxis, Double yaw,  boolean throttle) {
    return Commands.run(
() -> robotDrive.driveCartesian(yAxix/2, xAxis/2, yaw, m_gyro.getRotation2d()),driveSubsystem);
        
    // if (throttle)
    //     {
        
    //     robotDrive.driveCartesian(yAxix/10, xAxis/10, yaw, m_gyro.getRotation2d());
        
    //     }
    //     else{
    //         // robotDrive.driveCartesian(yAxix.getAsDouble()/5, xAxis.getAsDouble()/5, yaw.getAsDouble(), m_gyro.getRotation2d());

    //     }
  
    
    }

}