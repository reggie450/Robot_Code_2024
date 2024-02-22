package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 m_ColorV3 = new ColorSensorV3(i2cPort);
  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;
  private boolean collected = false;
  private boolean firing = false;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushless);
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kCoast);

    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Intake.kPositionGains);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_motor.burnFlash();

    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = 0.0;
  }

  /**
   * Set the power to spin the motor at.
   * This only applies outside of position mode.
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _power) {
    _power *= .3;
    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = _power;
  }

  /*
   * When the arm is in collection position, run it until payload is collected.
   * todo get retract working
   */
  public void collectPayload(double armEncode, boolean override_arm) {
    boolean armInPosition = armEncode >= Constants.Arm.kIntakePosition;

    Color detectedColor = m_ColorV3.getColor();
    double IR = m_ColorV3.getIR();
    int proximity = m_ColorV3.getProximity();

    boolean ispayloadPresent = getIsPayloadPresent(detectedColor, IR, proximity);
    if (!collected && !ispayloadPresent && (armInPosition || override_arm))
      setPower(.5);
    else if (!collected && ispayloadPresent) {
      m_motor.set(0.0);
      collected = true;
      retract();
    }
    SmartDashboard.putBoolean("Stop",ispayloadPresent);
}

  /**
   * The retract command should run immediately after the payload is detected. 
   * It will hold the payload back from the launcher.
   * todo make work
   * todo calibrate
   */
  public Command retract() {
    Command newCommand =
        new Command() {
          @Override
          public void initialize() {
            m_positionMode = true;
            m_targetPosition = m_encoder.getPosition() - Constants.Intake.kRetractDistance;
          }

          @Override
          public boolean isFinished() {
            return isNearTarget();
          }
        };

    newCommand.addRequirements(this);

    return newCommand;
  }

  /**
   * Will feed payload (orange ring) into launcher when command is given
   * Also resets collected status to false.
   * todo make work
   * todo calibrate
   */
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

  private boolean getIsPayloadPresent(Color detectedColor, double IR, double proximity) {
    return detectedColor.red >= .45 && detectedColor.red <= .75 &&
                               detectedColor.green >= .25 && detectedColor.green <= .45 &&
                               detectedColor.blue <= .15 && proximity > 300;
  }

  private void printColorValues(Color detectedColor, double IR, double proximity) {
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putBoolean("collected", collected);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    Color detectedColor = m_ColorV3.getColor();

    double IR = m_ColorV3.getIR();

    int proximity = m_ColorV3.getProximity();

    boolean ispayloadPresent = getIsPayloadPresent(detectedColor, IR, proximity);
    if (!collected && !firing && ispayloadPresent) {
      m_motor.set(0.0);
      collected = true;
      retract();      
    }

    // if we've reached the position target, drop out of position mode
    if (m_positionMode && isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
    }

    // update the motor power based on mode and setpoint
    if (m_positionMode) {
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
    }
    else { // Regular motion
      if (!collected || firing)
        m_motor.set(m_power);
      else
        m_motor.set(0);
    }

    // Record useful data
    printColorValues(detectedColor, IR, proximity);
    SmartDashboard.putBoolean("Stop",ispayloadPresent);

    SmartDashboard.putBoolean("Position Mode", m_positionMode);
    SmartDashboard.putBoolean("Near Target", isNearTarget());
    SmartDashboard.putNumber("IntakePower", m_power);

  }

  /**
   * Check if the encoder is within the position tolerance.
   * @return Whether the position is within the tolerance.
   */
  public boolean isNearTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }
}
