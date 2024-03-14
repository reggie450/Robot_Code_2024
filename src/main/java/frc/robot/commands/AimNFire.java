package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Launch.ShotType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AimNFire extends SequentialCommandGroup {
  public AimNFire(ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake, double setpoint){
    addCommands(
      new GoToTarget(arm, setpoint),
      new Launch(launcher,intake, ShotType.speakerShot, false));
  }

    
  private class GoToTarget extends Command {
    private ArmSubsystem s_arm;
    private double _setpoint;
    private Timer m_timer;
    private double elapsedTime = 0.0;
    private boolean blocked = false;

    public GoToTarget(ArmSubsystem arm, double setpoint){
      s_arm = arm;
      _setpoint = setpoint;
    }

    @Override
    public void initialize() {
      if (s_arm.running){
        blocked = true;
        return;
      }
      s_arm.setTargetPosition(_setpoint);
      s_arm.running = true;
      m_timer = new Timer();
      m_timer.start();
    }

    @Override
    public void execute() {
      if (!blocked){
      elapsedTime = m_timer.get();
      s_arm.runAutomatic(elapsedTime);
      }
    }

    @Override
    public boolean isFinished() {
      return s_arm.autoIsFinished(elapsedTime) || blocked;
    }

    @Override
    public void end(boolean interrupted) {
      if (!blocked){
        s_arm.stop();
        s_arm.running = false;
      }
    }
  }

}
