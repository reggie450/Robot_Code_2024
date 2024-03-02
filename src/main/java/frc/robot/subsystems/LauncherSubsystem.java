package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
  
  public enum ShotType{
    ampShot,
    speakerShot,
    owenWilsonSucks
  }

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
   * Turns the launcher off. Can be run once and the launcher will stay running or
   * run continuously in a {@code RunCommand}.
   */
  public void stopLauncher() {
  }

  public void primeShot(ShotType shotType) {
    double speed = .5;
    if (shotType == ShotType.ampShot){
      speed = .15;
    }
    else if (shotType == ShotType.speakerShot){
      speed = .5;
    }
    else if (shotType == ShotType.owenWilsonSucks) {
      speed = .7;
    }
    m_topMotor.set(speed);
    m_bottomMotor.set(speed - .05);
   //IntakeSubsystem.intakeRun(.3);
  }

  public void stopShooter() {
    m_topMotor.stopMotor();
    //Timer.delay(.1);
    m_bottomMotor.stopMotor();
    //Timer.delay(.1);
    // IntakeSubsystem.stopIntakeMotor();
  }

  public Command ShotIntake(IntakeSubsystem intake, ShotType shotType) {
    Command newCommand =
    new Command() {
      private Timer m_timer;

      @Override
      public void initialize() {
        m_timer = new Timer();
        m_timer.start();            
      }

      @Override
      public void execute() {
        Shot(shotType);
        if (m_timer.get() > Constants.Intake.kShotFeedTime)
          intake.run(.8);
      }

      @Override
      public boolean isFinished() {
        return m_timer.get() > .8;
      }

      @Override
      public void end(boolean interrupted) {
        intake.stop();
        stopShooter();
      }
    };

    newCommand.addRequirements(this, intake);

    return newCommand;
  }

  /* Autos */
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
