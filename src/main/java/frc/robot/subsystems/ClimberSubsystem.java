// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX m_leftmotor;
  private TalonFX m_rightmotor;
  // private double m_setpoint;

  // private TrapezoidProfile m_profile;
  private Timer m_timer;
  // private TrapezoidProfile.State m_startState;
  // private TrapezoidProfile.State m_endState;

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
  // private void updateMotionProfile() {
  //   m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
  //   //todo m_profile = new TrapezoidProfile(Constants.Climber.kClimberMotionConstraint);
  //   m_timer.reset();
  // }

  public void LeftUp(){
    m_leftmotor.set(Constants.Climber.kClimberSpeed);
  }
  public void RightUp(){
    m_rightmotor.set(Constants.Climber.kClimberSpeed);
  }
  public void LeftDown(){
    m_leftmotor.set(-Constants.Climber.kClimberSpeed);  
  }
  public void RightDown(){
    m_rightmotor.set(-Constants.Climber.kClimberSpeed);
  }
  public void LeftStop(){
    m_leftmotor.set(0);
  }
  public void RightStop() {
    m_rightmotor.set(0);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }
}
