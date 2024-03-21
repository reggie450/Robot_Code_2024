package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToTarget extends Command{
    private ArmSubsystem s_arm;
    private double _setpoint;
    private Timer m_timer;

    public ArmGoToTarget(ArmSubsystem arm, double setpoint){
      s_arm = arm;
      _setpoint = setpoint;
      double rotations = SmartDashboard.getNumber("Set Rotations", 0);
      _setpoint = rotations;
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      s_arm.running = true;
      m_timer = new Timer();
      m_timer.start();
    }

    @Override
    public void execute() {
      s_arm.SetReference(_setpoint);
    }

    @Override
    public boolean isFinished() {
      //where we are: s_arm.getEncoderPosition()
      // where we want to be: _setpoint
      // How close is close enough: Error
      return Math.abs(_setpoint - s_arm.getEncoderPosition()) < .001;
    }

    @Override
    public void end(boolean interrupted) {
        s_arm.stop();
        s_arm.running = false;
    }

}
