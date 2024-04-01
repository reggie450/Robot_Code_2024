// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  
  private DigitalInput inLimit = new DigitalInput(1);
  private int timesRun = 0;
  public boolean collected = false;
  public boolean running = false;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushless);
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kCoast);

    m_motor.burnFlash();

  }


  /*
   * This is the simple command for intake use with RunCommand
   */
  public void intakeSimple(){
    //SmartDashboard.putNumber("intakeSimpleRuns", timesRun++);
    if (!getLimitSwitch() && !collected)// && (!m_armBased || s_arm.getEncoderPosition() >= Constants.Arm.kIntakePosition))
      m_motor.set(.5);
    else {
      //SmartDashboard.putBoolean("intakeSimpleDidStop", true);
      m_motor.set(0);
      collected = true;
    }
  }  

  /*
   * Gets the value of the Limit Switch True means active
   */
  public boolean getLimitSwitch(){
    return inLimit.get();
  }

  /*
   * Stops the motor
   */
  public void stop(){
    m_motor.stopMotor();
  }

  /*
   * Runs the motor at the specified speed
   */
  public void run(double speed){   
    m_motor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeLimit", getLimitSwitch());
    SmartDashboard.putBoolean("IntakeCollected", collected);
    SmartDashboard.putBoolean("IntakeRunning", running);
  }
}
