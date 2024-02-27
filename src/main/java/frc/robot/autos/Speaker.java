// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Speaker extends SequentialCommandGroup {

  /** Creates a new HighCone. */
  public Speaker(Swerve s_swerve, LauncherSubsystem m_launcher, IntakeSubsystem m_intake, ArmSubsystem m_arm) {

    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        new InstantCommand(() -> m_arm.armDown(.8), m_arm),
        new WaitCommand(1.4).withTimeout(1.4),
        new InstantCommand(() -> m_arm.armStop(), m_arm),
        new WaitCommand(.1), 
        new InstantCommand(() -> m_launcher.autoSpeakerShot(),m_launcher),
        new WaitCommand(.7).withTimeout(.8),
        new InstantCommand(() -> m_launcher.stopShooter(), m_launcher),
        new WaitCommand(.5),
        new InstantCommand(() -> m_arm.armUp(.7),m_arm),
        new WaitCommand(1),
        new InstantCommand(() -> m_arm.armStop(), m_arm),

        // Drive towards other note
        // todo: adjust distance
        new exampleAuto(s_swerve));

  }
}
