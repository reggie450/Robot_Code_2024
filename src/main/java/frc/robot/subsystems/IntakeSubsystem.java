// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {

  private static CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 m_ColorV3 = new ColorSensorV3(i2cPort);
  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;
  private boolean collected = false;
  private boolean firing = false;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushless);
    m_motor.setInverted(true);
   // m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kCoast);

    /* m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Intake.kPositionGains);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, false); */
    m_motor.burnFlash();


  }

  public void setPower(double _power) {
    _power *= .3;
    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = _power;
  }

   public Command feedLauncher(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            setPower(0.0);
            firing = true;
          }

          @Override
          public void execute() {
            _launcher.runLauncher();
            if (m_timer.get() > Constants.Intake.kShotFeedTime)
              setPower(1.0);
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedEnd;
          }

          @Override
          public void end(boolean interrupted) {
            setPower(0.0);
            collected = false;
            firing = false;
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }

  public static void intakeRun(double speed){
    m_motor.set(speed);
  }

  public static void intakeStop(){
    m_motor.set(-.2);
    Timer.delay(.1);

    m_motor.stopMotor();
  }

  public static void stopIntakeMotor(){
    m_motor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
