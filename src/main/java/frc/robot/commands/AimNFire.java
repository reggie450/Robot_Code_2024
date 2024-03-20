package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Launch.ShotType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AimNFire extends SequentialCommandGroup {
  public AimNFire(ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake, double setpoint){
    addCommands(
      new ArmGoToTarget(arm, setpoint),
      new Launch(launcher,intake, ShotType.speakerShot, false));
  }
}
