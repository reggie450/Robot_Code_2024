// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_leadmotor;
  private CANSparkMax m_followmotor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // create a new SPARK MAX and configure it
    m_leadmotor = new CANSparkMax(Constants.Climber.kClimberCanId, MotorType.kBrushless);
    m_leadmotor.setInverted(false);
    //todo m_leadmotor.setSmartCurrentLimit(Constants.Climber.kCurrentLimit);
    m_leadmotor.setIdleMode(IdleMode.kBrake);
    m_leadmotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_leadmotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    //todo m_leadmotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Climber.kSoftLimitForward);
    //todo m_leadmotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Climber.kSoftLimitReverse);

    //todo m_followmotor = new CANSparkMax(Constants.Climber.kClimberFollowerCanId, MotorType.kBrushless);
    //todo m_followmotor.setSmartCurrentLimit(Constants.Climber.kCurrentLimit);
    m_followmotor.setInverted(true);
    m_followmotor.setIdleMode(IdleMode.kBrake);


    // set up the motor encoder including conversion factors to convert to radians and radians per second for position and velocity
    m_encoder = m_leadmotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    //todo m_encoder.setPositionConversionFactor(Constants.Climber.kPositionFactor);
    //todo m_encoder.setVelocityConversionFactor(Constants.Climber.kVelocityFactor);
    m_encoder.setPosition(0.0);

    m_controller = m_leadmotor.getPIDController();
    //todo PIDGains.setSparkMaxGains(m_controller, Constants.Climber.kClimberPositionGains);
    m_followmotor.follow(m_leadmotor,true);

    m_leadmotor.burnFlash();
    m_followmotor.burnFlash();


    //todo m_setpoint = Constants.Climber.kHomePosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   * @param _setpoint The new target position in radians.
  */
  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
  private void updateMotionProfile() {
    m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    //todo m_profile = new TrapezoidProfile(Constants.Climber.kClimberMotionConstraint);
    m_timer.reset();
  }

  /**
   * Drives the Climber to a position using a trapezoidal motion profile.
   * This function is usually wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   * <p>
   * This function updates the motor position control loop using a setpoint from the trapezoidal motion profile.
   * The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    //todo m_feedforward =
        //todo Constants.Climber.kClimberFeedforward.calculate(
            //todo m_encoder.getPosition() + Constants.Climber.kClimberZeroCosineOffset, m_targetState.velocity);
    //todo m_controller.setReference(
        //todo m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  /**
   * Drives the Climber using the provided power value (usually from a joystick).
   * This also adds in the feedforward value which can help counteract gravity.
   * @param _power The motor power to apply.
   */
  public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        //todo Constants.Climber.kClimberFeedforward.calculate(
            //todo m_encoder.getPosition() + Constants.Climber.kClimberZeroCosineOffset, m_targetState.velocity);
    // set the power of the motor
    //todo m_leadmotor.set(_power + (m_feedforward / 12.0));
    m_manualValue = _power; // this variable is only used for logging or debugging if needed
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }
}
