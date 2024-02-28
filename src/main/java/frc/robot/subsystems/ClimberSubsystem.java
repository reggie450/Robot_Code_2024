// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX m_leftmotor;
  private TalonFX m_rightmotor;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // create a new SPARK MAX and configure it
    m_leftmotor = new TalonFX(Constants.Climber.kClimberLeftCanId);
    m_leftmotor.setInverted(false);

    m_rightmotor = new TalonFX(Constants.Climber.kClimberRightCanId);
    m_rightmotor.setInverted(true);

    m_timer = new Timer();
    m_timer.start();

    //updateMotionProfile();
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   * @param _setpoint The new target position in radians.
  */
 /*  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    } */
  

  /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
  private void updateMotionProfile() {
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    //todo m_profile = new TrapezoidProfile(Constants.Climber.kClimberMotionConstraint);
    m_timer.reset();
  }

  public void climbUpLeft(){
    m_leftmotor.set(Constants.Climber.kClimberSpeed);
  }
  public void climbUpRight(){
  m_rightmotor.set(Constants.Climber.kClimberSpeed);
  }
  public void climbDownLeft(){
  m_leftmotor.set(-Constants.Climber.kClimberSpeed);  
  }
  public void climbDownRight(){
  m_rightmotor.set(-Constants.Climber.kClimberSpeed);
  }
  public void climbStopLeft(){
  m_leftmotor.set(0);
  }
  public void climbStopRight() {
  m_rightmotor.set(0);
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
    // if (m_profile.isFinished(elapsedTime)) {
    //   m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    // } else {
    //   m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    // }

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
    // m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }
}
