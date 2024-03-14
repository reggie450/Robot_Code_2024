package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Launch extends Command {
  public enum ShotType{
    ampShot,
    speakerShot,
    owenWilsonSucks,
    autoSpeakerShot
  }

  private LauncherSubsystem s_launcher;
  private IntakeSubsystem s_intake;
  private ShotType m_shotType;
  private boolean m_keepOn;

  private Timer m_timer;

  public Launch(LauncherSubsystem launcher, IntakeSubsystem intake, ShotType shotType, Boolean keepOn) {
    s_launcher = launcher;
    s_intake = intake;
    m_shotType = shotType;
    m_keepOn = keepOn;
    addRequirements(s_launcher, s_intake);
  }


  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();            
    SmartDashboard.putBoolean("LauncherCommandRun - Debug", true);
  }

  @Override
  public void execute() {
    primeShot(m_shotType,0.0);
    if (m_timer.get() > Constants.Intake.kShotFeedTime)
      s_intake.run(.8);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > .8;
  }

  @Override
  public void end(boolean interrupted) {
    s_intake.stop();
    if (!m_keepOn){
      s_launcher.stop();
    }
    s_intake.collected = false;
    SmartDashboard.putBoolean("LauncherCommandRun - Debug", false);

  }

  private void primeShot(ShotType shotType, double adjustment) {
    if (adjustment > .2 ) adjustment = .2;
    if (adjustment < -.2 ) adjustment = -.2;

    double topSpeed = .5;
    double bottomSpeed = .5;
    if (shotType == ShotType.ampShot){
      topSpeed = .15;
      bottomSpeed = topSpeed - .05;
    }
    else if (shotType == ShotType.speakerShot){
      topSpeed = .5;
      bottomSpeed = topSpeed - .05;
    }
    else if (shotType == ShotType.owenWilsonSucks) {
      topSpeed = .7;
      bottomSpeed = topSpeed - .05;
    }
    else if (shotType == ShotType.autoSpeakerShot){
      topSpeed = .5;
      bottomSpeed = .5;
    }

    s_launcher.run(topSpeed+adjustment,bottomSpeed+adjustment);
  }
  

}
