package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoDown extends Command{
    private ArmSubsystem s_arm;
    private Timer m_timer;

    public ArmGoDown(ArmSubsystem arm){
      s_arm = arm;
      //double rotations = SmartDashboard.getNumber("Set Rotations", 0);
      //_setpoint = rotations;
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      s_arm.running = true;
      m_timer = new Timer();
      m_timer.start();
      //s_setpoint = SmartDashboard.getNumber("Set Rotations", 0);
    }

    @Override
    public void execute() {
      s_arm.down(1.0);
    }

    @Override
    public boolean isFinished() {
      //where we are: s_arm.getEncoderPosition()
      // where we want to be: _setpoint
      // How close is close enough: Error
      return s_arm.isDown() || m_timer.get() > 1; //Math.abs(_setpoint - s_arm.getEncoderPosition()) < .01;
    }

    @Override
    public void end(boolean interrupted) {
        s_arm.stop();
        s_arm.running = false;
    }

}
