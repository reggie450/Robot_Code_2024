package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /* 
   * Beware of using May cause motors to not work
   *  */
  public void PlaySong(){
    Orchestra orchestra = new Orchestra();
    orchestra.addInstrument(m_topMotor);
    orchestra.addInstrument(m_bottomMotor);
    var status = orchestra.loadMusic("/home/lvuser/test.chrp");
    if (!status.isOK()) {
      SmartDashboard.putString("MusicLoadError", "Music Not Worky");
    }
    status = orchestra.play();
    if (!status.isOK()) {
      SmartDashboard.putString("MusicPlayError", "Music Not Worky");
    }
    orchestra.close();
  } 

  /**
   * Turns the launcher off. Can be run once and the launcher will stop
   *  in a {@code RunCommand}.
   */
  public void stop() {
    m_topMotor.stopMotor();
    m_bottomMotor.stopMotor();
  }

  /**
   * Sets Launcher Speeds. Can be run once and the launcher will stay running or
   * run continuously in a {@code RunCommand}.
   */
  public void run(double speedTop, double speedBottom) {
    m_topMotor.set(speedTop);
    m_bottomMotor.set(speedBottom);
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run

  }
}
