// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmGoToTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimTest extends SequentialCommandGroup {


  /** Creates a new Speakershot. */
  public AimTest(LauncherSubsystem launcher, IntakeSubsystem intake, ArmSubsystem arm) {
    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        // Go to Shot Location
        new ArmGoToTarget(arm, -1.0)
        // new InstantCommand(() -> arm.down(.8), arm), 
        // new WaitCommand(.95).withTimeout(.95),
        // new InstantCommand(() -> arm.stop(), arm),
  
        // Launch
        //new Launch(launcher, intake, ShotType.autoSpeakerShot, keepRunning)      
    );
  }
}
