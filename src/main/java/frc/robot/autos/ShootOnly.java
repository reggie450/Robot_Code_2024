// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.autos.SpeakerRight.TraverseBack;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOnly extends SequentialCommandGroup {


  /** Creates a new Speakershot. */
  public ShootOnly(Swerve s_swerve, LauncherSubsystem m_launcher, IntakeSubsystem m_intake, ArmSubsystem m_arm) {
    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        new InstantCommand(() -> m_arm.armDown(.8), m_arm),
        new WaitCommand(1).withTimeout(1),
        new InstantCommand(() -> m_arm.armStop(), m_arm),
        new WaitCommand(.2), 
        new InstantCommand(() -> m_launcher.autoSpeakerShot(),m_launcher),
        new WaitCommand(.4).withTimeout(.5),
        new InstantCommand(() -> m_launcher.stopShooter(), m_launcher),
        new InstantCommand(() -> IntakeSubsystem.intakeStop()),
        //new InstantCommand(() -> IntakeSubsystem.intakeRun(.5), m_intake),
        new InstantCommand(() -> m_arm.armDown(.9)),
        new WaitCommand(.2)
        // new InstantCommand(() -> m_arm.armUp(.7),m_arm),
        // new WaitCommand(1),
        // new InstantCommand(() -> m_arm.armStop(), m_arm),

        // Drive towards other note
        // todo: adjust distance
        // traverse,
        // new WaitCommand(.2),
        // new InstantCommand(()->IntakeSubsystem.intakeStop()),
        // //new TraverseBack(s_swerve),
        // new InstantCommand(() -> m_arm.armUp(.8)),
        // new WaitCommand(.3),
        // new InstantCommand(() -> m_arm.armStop(), m_arm),
        // new WaitCommand(.05),
        // new InstantCommand(()->IntakeSubsystem.intakebackup()),
        // //new WaitCommand(.01).withTimeout(.01),
        // new InstantCommand(() -> m_launcher.autoSpeakerShot(),m_launcher),
        // new WaitCommand(.4).withTimeout(.4),
        // new InstantCommand(() -> m_launcher.stopShooter(), m_launcher),
        // new InstantCommand(()->IntakeSubsystem.intakeStop())
    );

  }
}
