// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Launch;
import frc.robot.commands.Launch.ShotType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOnly extends SequentialCommandGroup {


  /** Creates a new Speakershot. */
  public ShootOnly(Swerve swerve, LauncherSubsystem launcher, IntakeSubsystem intake, ArmSubsystem arm) {
    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        new InstantCommand(() -> arm.down(.8), arm),
        new WaitCommand(1).withTimeout(1),
        new InstantCommand(() -> arm.stop(), arm),
        new WaitCommand(.2), 
        new Launch(launcher, intake, ShotType.autoSpeakerShot, false),
        new InstantCommand(() -> arm.down(.9)),
        new WaitCommand(.2)
    );

  }
}
