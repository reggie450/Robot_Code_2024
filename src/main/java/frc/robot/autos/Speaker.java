// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmGoDown;
import frc.robot.commands.ArmGoToTarget;
import frc.robot.commands.AutoCollect;
import frc.robot.commands.Launch;
import frc.robot.commands.Launch.ShotType;
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
    double armWaitTime = .58;
    double powerAdjust = .05;
    // if (!SmartDashboard.containsKey("armWaitTime"))
    //   SmartDashboard.putNumber("armWaitTime",armWaitTime);
    // else
    //   SmartDashboard.getNumber("armWaitTime", .6);
    // if (!SmartDashboard.containsKey("armWaitTime"))
    //   SmartDashboard.putNumber("powerAdjust",powerAdjust);
    // else
    //   SmartDashboard.getNumber("powerAdjust", .05);
    addCommands(
        new ShootOnly(launcher, intake, arm, true),
        
        // Collect
        new ArmGoDown(arm),
        //new WaitCommand(.4)
        new AutoCollect(traverse, intake), 
        // new TraverseBack(s_swerve),

        // Go to Launch Position
        new ArmGoToTarget(arm, 1),
        // Launch
        new Launch(launcher, intake, ShotType.autoSpeakerShot, false, powerAdjust)
    );

  }
}
