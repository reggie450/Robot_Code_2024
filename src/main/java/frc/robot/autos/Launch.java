package frc.robot.autos;

import frc.robot.Constants.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class Launch {

    Swerve s_swerve;
    ArmSubsystem m_arm;
    LauncherSubsystem m_launcher;
    IntakeSubsystem m_intake;
    ClimberSubsystem m_climber;
    public Launch(Swerve swerve, ArmSubsystem arm, LauncherSubsystem launcher, IntakeSubsystem intake, ClimberSubsystem climber) {
        s_swerve = swerve;
    }
}
