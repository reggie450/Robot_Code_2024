package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToTarget extends Command{
    private ArmSubsystem s_arm;
    private double _setpoint;
    private Timer m_timer;
    private double elapsedTime = 0.0;
    private boolean blocked = false;

    public ArmGoToTarget(ArmSubsystem arm, double setpoint){
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
      return blocked || s_arm.autoIsFinished(elapsedTime);
    }

    @Override
    public void end(boolean interrupted) {
      if (!blocked){
        s_arm.stop();
        s_arm.running = false;
      }
    }

}
