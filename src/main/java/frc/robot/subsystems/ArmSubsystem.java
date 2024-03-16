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

  }

  /**
   * Sets the target position and updates the motion profile if the target
   * position changed.
   * 
   * @param _setpoint The new target position in radians.
   */
  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  /**
   * Update the motion profile variables based on the current setpoint and the
   * pre-configured motion constraints.
   */
   private void updateMotionProfile() {
    m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
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

  public void down(double speed) {
    m_leadmotor.set(speed);
  }

  public void stop() {
    m_leadmotor.stopMotor();
  }

  public void up(double speed) {
    m_leadmotor.set(-speed);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    //stats.Periodic();
    // SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    // SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
    SmartDashboard.putNumber("Arm Encoder", m_encoder.getPosition());
  }
}
