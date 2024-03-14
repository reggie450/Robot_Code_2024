package frc.robot.commands;

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
        public GoToTarget(ArmSubsystem arm, double setpoint){
            s_arm = arm;
            _setpoint = setpoint;
        }

        @Override
        public void initialize() {
          s_arm.setTargetPosition(_setpoint);
          s_arm.running = true;
        }

        @Override
        public void execute() {
            s_arm.runAutomatic();
        }

        @Override
        public boolean isFinished() {
          return s_arm.autoIsFinished();
        }

        @Override
        public void end(boolean interrupted) {
          s_arm.stop();
          s_arm.running = false;
        }
      }

}
