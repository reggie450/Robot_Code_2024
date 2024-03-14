// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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


  /** Creates a new Speakershot. */
  public Speaker(Swerve swerve, LauncherSubsystem launcher, IntakeSubsystem intake, ArmSubsystem arm, Command traverse, Command traverseBack) {
    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    double armWaitTime = .6;
    double powerAdjust = .05;
    if (!SmartDashboard.containsKey("armWaitTime"))
      SmartDashboard.putNumber("armWaitTime",armWaitTime);
    else
      SmartDashboard.getNumber("armWaitTime", .6);
    if (!SmartDashboard.containsKey("armWaitTime"))
      SmartDashboard.putNumber("powerAdjust",powerAdjust);
    else
      SmartDashboard.getNumber("powerAdjust", .05);
    addCommands(
        new InstantCommand(() -> arm.armDown(.8), arm),
        new WaitCommand(1).withTimeout(1),
        new InstantCommand(() -> arm.armStop(), arm),
        new WaitCommand(.2), 
        new InstantCommand(() -> launcher.autoSpeakerShot(intake),launcher),
        new WaitCommand(.4).withTimeout(.5),
       // new InstantCommand(() -> launcher.stopShooter(), launcher), Grenier
        new InstantCommand(() -> intake.run(.5), intake), 
        new InstantCommand(() -> arm.armDown(.9)),
        new WaitCommand(.2),
        // new InstantCommand(() -> arm.armUp(.7),arm),
        // new WaitCommand(1),
        // new InstantCommand(() -> arm.armStop(), arm),

        // Drive towards other note
        // todo: adjust distance
        traverse,
        new WaitCommand(.2),
       // new InstantCommand(()->intake.stop()), Grenier
        //new TraverseBack(s_swerve),
        new InstantCommand(() -> arm.armUp(1)),
        new WaitCommand(armWaitTime), //.58 too high
        new InstantCommand(() -> arm.armStop(), arm),
        new WaitCommand(.05),
      //  new InstantCommand(()->intake.backup()), Grenier
        new WaitCommand(.01).withTimeout(.01),
        //new InstantCommand(()->intake.stop()), Grenier
        new InstantCommand(() -> launcher.autoSpeakerShot(intake,powerAdjust),launcher),
        new WaitCommand(.4).withTimeout(.4),
        new InstantCommand(() -> launcher.stop(), launcher),
        new InstantCommand(()->intake.stop())
    );

  }
}
