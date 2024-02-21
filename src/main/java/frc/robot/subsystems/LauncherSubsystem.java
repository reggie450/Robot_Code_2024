package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private TalonFX m_topMotor;
  private TalonFX m_bottomMotor;

  private boolean m_launcherRunning;

  /**
   * Creates a new LauncherSubsystem.
   */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor = new TalonFX(Constants.Launcher.kTopCanId);
    m_topMotor.setInverted(false);


    m_bottomMotor = new TalonFX(Constants.Launcher.kBottomCanId);
    m_bottomMotor.setInverted(false);

    

    m_launcherRunning = false;
  }

  /**
   * Turns the launcher on.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() {  // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      m_topMotor.set(Constants.Launcher.kTopPower);
      m_bottomMotor.set(Constants.Launcher.kBottomPower);
    } else {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
    }
  }
}
