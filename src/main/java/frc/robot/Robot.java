// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.lang.StrictMath;
import java.util.TimerTask;
import java.util.function.BooleanSupplier;

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
  private static final int kIntakeLeftChannel = 6;
  private static final int kIntakeRightChannel = 7;
  private static final int kLeftTemp = 0;
  private static final int kRightTemp = 1;
  private static final int kDriverController = 0;
  private static final int kOperatorController = 1;
  private static final int kButtonTurbo = Constants.ControllerConstants.RT;
  private static final int kButtonGyroAdjust = Constants.ControllerConstants.LT;
  private static final int kButtonGyroReset = Constants.ControllerConstants.BACK;
  private static final int kButtonLiftUp = Constants.ControllerConstants.Y;
  private static final int kButtonLiftDown = Constants.ControllerConstants.A;
  private static final int kButtonActUp = Constants.ControllerConstants.RB;
  private static final int kButtonActDown = Constants.ControllerConstants.LB;
  private static final int kButtonActSelect = Constants.ControllerConstants.B;
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
  private static SparkMax intakeLeft;
  private static SparkMax intakeRight;
  private Servo leftAct;
  private Servo rightAct;
  private double leftpos;
  private double rightpos;
  private double autoxaxis;
  private double autoyaxis;
  private double autorot;
  private Timer timer;
  private commandStream commandQueue;
  private boolean working;
  private RelativeEncoder frontLeftEncoder;
  public Robot() {
    commandQueue = new commandStream();
    working = false;
    timer = new Timer();
    SparkMax frontLeft = new SparkMax(kFrontLeftChannel, MotorType.kBrushless);
    SparkMax rearLeft = new SparkMax(kRearLeftChannel, MotorType.kBrushless);
    SparkMax frontRight = new SparkMax(kFrontRightChannel, MotorType.kBrushless);
    SparkMax rearRight = new SparkMax(kRearRightChannel, MotorType.kBrushless);
    frontLeftEncoder = frontLeft.getEncoder();
    frontLeftEncoder.setPosition(0);
    leftAct = new Servo(kLeftTemp);
    rightAct = new Servo(kRightTemp);
    lift = new SparkMax(kLiftChannel, MotorType.kBrushless);
    intakeLeft = new SparkMax(kIntakeLeftChannel, MotorType.kBrushless);
    intakeRight = new SparkMax(kIntakeRightChannel, MotorType.kBrushless);
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
    liftEncoder.setPosition(0);
    leftpos = 0;
    rightpos = 0;
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
    // will go from 0 to 400
    // double speed = (m_operator.getRawButton(kButtonLiftUp) ? 1.0 : 0.0) * StrictMath.exp(-liftEncoder.getPosition() / 20) - (m_operator.getRawButton(kButtonLiftDown) ? 1.0 : 0.0) * StrictMath.exp(liftEncoder.getPosition() / 20);
    // double speed = (m_operator.getRawButton(kButtonLiftUp) ? 1.0 : 0.0) - (m_operator.getRawButton(kButtonLiftDown) ? 1.0 : 0.0);
    double speed = -m_operator.getRawAxis(1)*(0.2-liftEncoder.getPosition()*(liftEncoder.getPosition()/50000-0.008));
    if(speed > 1.0) {
      speed = 1.0;
    }
    if(speed < -1.0) {
      speed = -1.0;
    }
    lift.set(speed);
    speed = -m_operator.getRawAxis(3);
    if(speed > 1.0) {
      speed = 1.0;
    }
    if(speed < -1.0) {
      speed = -1.0;
    }
    intakeLeft.set(speed);
    intakeRight.set(-speed);
    speed = (m_operator.getRawButton(kButtonActUp) ? 1.0 : 0.0) - (m_operator.getRawButton(kButtonActDown) ? 1.0 : 0.0);
    if(speed > 1.0) {
      speed = 1.0;
    }
    if(speed < -1.0) {
      speed = -1.0;
    }
    if(m_operator.getRawButton(kButtonActSelect)) {
      leftpos += speed / 100;
      leftAct.set(leftpos);
    } else {
      rightpos += speed / 100;
      rightAct.set(rightpos);
    }
    // double target = 50 * (m_operator.getRawButton(kButtonLiftUp) ? 1.0 : 0.0) + 25 * (m_operator.getRawButton(kButtonLiftDown) ? 1.0 : 0.0);
    // lift.set(StrictMath.exp(-(liftEncoder.getPosition() - target) / 20) - StrictMath.exp((liftEncoder.getPosition() - target) / 20));
    gyroMeasure += gyroAdjust;
    double gyroAdjustedSideways = sideways * StrictMath.cos(gyroMeasure) + forwards * StrictMath.sin(gyroMeasure);
    double gyroAdjustedForwards = forwards * StrictMath.cos(gyroMeasure) - sideways * StrictMath.sin(gyroMeasure);
    m_robotDrive.driveCartesian(-rotation, -gyroAdjustedSideways, -gyroAdjustedForwards);
  }
  public class commandStream {
    private Queue<command> queue;
    public commandStream() {
        queue = new LinkedList<>();
    }
    public void deposit(command element) {
        queue.add(element);
    }
    public command withdrawl() {
        return queue.poll();
    }
    public boolean has() {
        return !queue.isEmpty();
    }
  }
  class task extends TimerTask {
    public command com;
    public task (command com) {
      this.com = com;
    }
    public void run () {
      this.com.schedule();
    }
  }
  class command extends TimerTask {
    public int type;
    public long wait;
    public BooleanSupplier cond;
    public command (long wait) {
      this.wait = wait;
      this.type = 0;
      commandQueue.deposit(this);
    }
    public command (BooleanSupplier conditionEvaluator) {
      this.cond = conditionEvaluator;
      this.type = 1;
      commandQueue.deposit(this);
    }
    public void act() {;}
    public void run() {
      if (type==0) {
        act();
      }
      working = false;
    }
    public void schedule() {
      if (type == 0) {
        timer.schedule(this, wait);
      } else if (type == 1) {
        if (this.cond.getAsBoolean()) {
          timer.schedule(this, 0);
        } else {
          timer.schedule(new task(this), 50);
        }
      }
    }
  }
  class xaxisset extends command {
    public double val;
    public xaxisset(long wait, double val) {
      super(wait);
      this.val = val;
    }
    public xaxisset(double val) {
      super(0);
      this.val = val;
    }
    public void act() {
      autoxaxis = this.val;
    }
  }
  class yaxisset extends command {
    public double val;
    public yaxisset(long wait, double val) {
      super(wait);
      this.val = val;
    }
    public yaxisset(double val) {
      super(0);
      this.val = val;
    }
    public void act() {
      autoyaxis = this.val;
    }
  }
  class rotset extends command {
    public double val;
    public rotset(long wait, double val) {
      super(wait);
      this.val = val;
    }
    public rotset(double val) {
      super(0);
      this.val = val;
    }
    public void act() {
      autorot = this.val;
    }
  }
  // #TODO
  // currently takes dist in units of rotations of the front left wheel.
  class gofordist extends command {
    public gofordist(double dist) {
      super(() -> {return false;});
      double startDist = frontLeftEncoder.getPosition();
      this.cond = () -> {return Math.abs(frontLeftEncoder.getPosition() - startDist) >= dist;};
    }
  }
  class genericCommand extends command {
    public Runnable action;
    public genericCommand(long wait, Runnable action) {
      super(wait);
      this.action = action;
    }
    public genericCommand(Runnable action) {
      super(0);
      this.action = action;
    }
    public void act() {
      this.action.run();
    }
  }
  // currently implemented commands:
  //   wait for specified amount of milliseconds
  //   wait for boolean to be true ( checks every 20th of a second )*
  //   set x velocity after a specified amount of milliseconds
  //   set x velocity immediately
  //   set y velocity after a specified amount of milliseconds
  //   set y velocity immediately
  //   set rotational velocity after a specified amount of milliseconds
  //   set rotational velocity immediately
  //   keep moving until specified amount of rotations of the front left wheel have occurred ( checks every 20th of a second )*
  //   run custom code after specified amount of milliseconds
  //   run custom code immediately
  // *every 20th of a second comes from the 50 ms break between checks in command.schedule() in the type 1 false branch
  @Override
  public void autonomousInit() {
    new yaxisset(1000, 1);
    new yaxisset(1000, 0);
    new yaxisset(1000, -1);
    new yaxisset(1000, 0);
    new command(() -> {return m_driver.getRawButton(Constants.ControllerConstants.X);});
    new rotset(0, 0.25);
    new gofordist(1);
    new rotset(0, -0.25);
    new gofordist(-1);
    new rotset(0, 0);
    autoxaxis = 0;
    autoyaxis = 0;
    autorot = 0;
  }
  @Override
  public void autonomousPeriodic() {
    if(!working && commandQueue.has()) {
      commandQueue.withdrawl().schedule();
      working = true;
    }
    m_robotDrive.driveCartesian(autorot, autoxaxis, autoyaxis);
  }
}