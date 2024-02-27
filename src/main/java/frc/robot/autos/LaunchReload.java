package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class LaunchReload {
    /* shoot reload shoot */
    // shoot
    // arm down
    // intake on
    // go forward
    // retract intake
    // arm up
    // shoot again

    Swerve s_swerve;
    ArmSubsystem m_arm;
    LauncherSubsystem m_launcher;
    IntakeSubsystem m_intake;
    ClimberSubsystem m_climber;
    public LaunchReload(Swerve swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake, ClimberSubsystem climber) {
        s_swerve = swerve;
    }
}
