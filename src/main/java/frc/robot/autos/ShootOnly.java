// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGoToTarget;
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
  public ShootOnly(LauncherSubsystem launcher, IntakeSubsystem intake, ArmSubsystem arm, boolean keepRunning) {
    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        // Go to Shot Location
        new ArmGoToTarget(arm, 1.45), // First Shot Adjustment
  
        // Launch
        new Launch(launcher, intake, ShotType.autoSpeakerShot, keepRunning,.1)      
    );
  }

  public ShootOnly(LauncherSubsystem launcher, IntakeSubsystem intake, ArmSubsystem arm) {
    this(launcher, intake, arm, false);
  }
}
