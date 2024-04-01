// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
//import frc.robot.StatsCollection;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_leadmotor;
  private CANSparkMax m_followmotor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  public boolean running = false;




  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  // private StatsCollection stats = new StatsCollection("ArmSS");
  /** Creates a new ArmSubsystem and sets default behaviors */
  public ArmSubsystem() {
    // create a new SPARK MAX and configure it
    m_leadmotor = new CANSparkMax(Constants.Arm.kArmCanId, MotorType.kBrushless);
    m_leadmotor.setInverted(false);
   // m_leadmotor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    m_leadmotor.setIdleMode(IdleMode.kBrake);
   // m_leadmotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    //m_leadmotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  //  m_leadmotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
 //   m_leadmotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);

    m_followmotor = new CANSparkMax(Constants.Arm.kArmFollowerCanId, MotorType.kBrushless);
   // m_followmotor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    m_followmotor.setInverted(true);
    m_followmotor.setIdleMode(IdleMode.kBrake);

    // set up the motor encoder including conversion factors to convert to radians
    // and radians per second for position and velocity
    m_encoder = m_leadmotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);
    //m_encoder.setPosition(0.0);

    m_controller = m_leadmotor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Arm.kArmPositionGains);
    m_followmotor.follow(m_leadmotor, true);

    m_leadmotor.burnFlash();
    m_followmotor.burnFlash();

    m_forwardLimit = m_leadmotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit = m_leadmotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_leadmotor.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    m_pidController.setFeedbackDevice(m_encoder);

    // PID coefficients
    kP = 1; 
    kI = 0; //1e-4;
    kD = 0; //1;
    kIz = 0; 
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

/**
   * Drives the arm to a position using a trapezoidal motion profile.
   * This function is usually wrapped in a {@code RunCommand} which runs it
   * repeatedly while the command is active.
   * <p>
   * This function updates the motor position control loop using a setpoint from
   * the trapezoidal motion profile.
   * The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic(double elapsedTime) {
    if (autoIsFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward = Constants.Arm.kArmFeedforward.calculate(
        m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    m_controller.setReference(
        m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);

  }

  public boolean autoIsFinished(double elapsedTime) {
    return m_profile.isFinished(elapsedTime);
  }


  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }
 
  /**
   * Drives the arm using the provided power value (usually from a joystick).
   * This also adds in the feedforward value which can help counteract gravity.
   * 
   * @param _power The motor power to apply.
   */
  public void runManual(double _power, double alternate) {
    // set the power of the motor
    if (Math.abs(_power) > Math.abs(alternate)) {
      m_manualValue = MathUtil.applyDeadband(_power, .1);;
    }
    else{
      m_manualValue = MathUtil.applyDeadband(alternate, .1);
    }
    m_leadmotor.set(m_manualValue);
  } 

  public boolean isDown() {
    return getEncoderPosition() > 1.73;
  }

  public void down(double speed) {
    m_leadmotor.set(speed);
  }

  public void stop() {
    m_leadmotor.stopMotor();
  }

  public void up(double speed) {
    m_leadmotor.set(-speed);
  }


  public void resetEncoder() {
    m_encoder.setPosition(0.0);
  }

  // public void setTarget(double _setpoint) {
  //   m_setpoint = _setpoint;
  // }

  public void SetReference (double rotations){
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  // public void autonomousPeriodic() {
  //   //stats.Periodic();
  //   // SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
  //   // SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
  //   SmartDashboard.putNumber("Arm Encoder", m_encoder.getPosition());
  //   // read PID coefficients from SmartDashboard
  //   double p = SmartDashboard.getNumber("P Gain", 0);
  //   double i = SmartDashboard.getNumber("I Gain", 0);
  //   double d = SmartDashboard.getNumber("D Gain", 0);
  //   double iz = SmartDashboard.getNumber("I Zone", 0);
  //   double ff = SmartDashboard.getNumber("Feed Forward", 0);
  //   double max = SmartDashboard.getNumber("Max Output", 0);
  //   double min = SmartDashboard.getNumber("Min Output", 0);
  //   double rotations = SmartDashboard.getNumber("Set Rotations", 0);

  //   // if PID coefficients on SmartDashboard have changed, write new values to controller
  //   if((p != kP)) { m_pidController.setP(p); kP = p; }
  //   if((i != kI)) { m_pidController.setI(i); kI = i; }
  //   if((d != kD)) { m_pidController.setD(d); kD = d; }
  //   if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
  //   if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
  //   if((max != kMaxOutput) || (min != kMinOutput)) { 
  //     m_pidController.setOutputRange(min, max); 
  //     kMinOutput = min; kMaxOutput = max; 
  //   }
  //   /**
  //    * PIDController objects are commanded to a set point using the 
  //    * SetReference() method.
  //    * 
  //    * The first parameter is the value of the set point, whose units vary
  //    * depending on the control type set in the second parameter.
  //    * 
  //    * The second parameter is the control type can be set to one of four 
  //    * parameters:
  //    *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
  //    *  com.revrobotics.CANSparkMax.ControlType.kPosition
  //    *  com.revrobotics.CANSparkMax.ControlType.kVelocity
  //    *  com.revrobotics.CANSparkMax.ControlType.kVoltage
  //    */
  
  //   SmartDashboard.putNumber("SetPoint", rotations);
  //   SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

  //   SetReference(m_setpoint);
  // }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    //stats.Periodic();
    // SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    // SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
    SmartDashboard.putNumber("Arm Encoder", m_encoder.getPosition());
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
  
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
