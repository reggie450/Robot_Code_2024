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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchReload extends SequentialCommandGroup {

  /** Creates a new HighCone. */
  public LaunchReload(Swerve s_swerve, LauncherSubsystem m_launcher, IntakeSubsystem m_intake, ArmSubsystem m_arm) {

    // Shots note into Speaker and then drives out of zone
    // todo adjust timings
    addCommands(
        new InstantCommand(() -> m_launcher.autoSpeakerShot(m_intake),m_launcher), // shoot
        new WaitCommand(.7).withTimeout(.8),
        new InstantCommand(() -> m_launcher.stopShooter(), m_launcher), 
        new WaitCommand(.5),
        new InstantCommand(() -> m_arm.armDown(.8), m_arm), // arm down
        new WaitCommand(1.4).withTimeout(1.4),
        new InstantCommand(() -> m_arm.armStop(), m_arm),
        new WaitCommand(.1), 
        new InstantCommand(() -> m_intake.intakeRun(1),m_intake), // intake on
        new InstantCommand(() -> s_swerve.driveForward(2), s_swerve), // go forward
        new WaitCommand(.1),
        new InstantCommand(() -> m_intake.intakeStop(),m_intake), // intake off
        // retract intake
        new InstantCommand(() -> m_arm.armUp(.7),m_arm), // arm up
        new WaitCommand(1),
        new InstantCommand(() -> m_arm.armStop(), m_arm),
        new InstantCommand(() -> m_launcher.autoSpeakerShot(m_intake),m_launcher), // shoot
        new WaitCommand(.7).withTimeout(.8),
        new InstantCommand(() -> m_launcher.stopShooter(), m_launcher), 
        new WaitCommand(.5)

        // Drive towards other note
        // todo: adjust distance
        //new exampleAuto(s_swerve)
        );

  }


  public void DriveTo(){
    TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(1, 0),

                    new Translation2d(1, 0)),
            // End 5 meters straight behind of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

  }


    // public exampleAuto(Swerve s_Swerve) {



    //     var thetaController = new ProfiledPIDController(
    //             Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //             exampleTrajectory,
    //             s_Swerve::getPose,
    //             Constants.Swerve.swerveKinematics,
    //             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //             thetaController,
    //             s_Swerve::setModuleStates,
    //             s_Swerve);

    //     addCommands(
    //             new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
    //             swerveControllerCommand);

    // }
}