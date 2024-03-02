package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private TalonFX m_topMotor;
  private TalonFX m_bottomMotor;

  /**
   * Creates a new LauncherSubsystem.
   */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor = new TalonFX(Constants.Launcher.kTopCanId);
    m_topMotor.setInverted(false);
    m_topMotor.setNeutralMode(NeutralModeValue.Brake);

    m_bottomMotor = new TalonFX(Constants.Launcher.kBottomCanId);
    m_bottomMotor.setInverted(false); 
    m_bottomMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or
   * run continuously in a {@code RunCommand}.
   */
  public void runLauncher() {
  }

  public void owenWilsonSucks() {
    m_topMotor.set(.7);
    m_bottomMotor.set(.6);
  /*  Timer.delay(.5);
   IntakeSubsystem.intakeRun(.5); */
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or
   * run continuously in a {@code RunCommand}.
   */
  public void stopLauncher() {
  }

  public void ampShot() {
    m_topMotor.set(.15);
    m_bottomMotor.set(.15);
   //IntakeSubsystem.intakeRun(.3);
  }

  public void stopShooter() {
    m_topMotor.stopMotor();
    //Timer.delay(.1);
    m_bottomMotor.stopMotor();
    //Timer.delay(.1);
    // IntakeSubsystem.stopIntakeMotor();
  }
    

  public void speakerShot() {
    m_topMotor.set(.5);
    m_bottomMotor.set(.4);
    // Timer.delay(.2);
    // IntakeSubsystem.intakeRun(.7);
  }

  public void autoSpeakerShot(IntakeSubsystem intake) {
    autoSpeakerShot(intake, 0.0);
  }

  public void autoSpeakerShot(IntakeSubsystem intake, double adjustment) {
    if (adjustment > .2 ) adjustment = .2;
    m_topMotor.set(.5+adjustment);
    m_bottomMotor.set(.5+adjustment);
    Timer.delay(.7);
    intake.run(.8+adjustment);
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run

  }
}
