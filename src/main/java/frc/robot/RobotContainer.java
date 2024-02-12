// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
//import frc.utils.GamepadUtils;
import java.util.List;

/*
* This Class contains commands and button maps which call methods in the subsystems
*/
public class RobotContainer {
    // The Weapon subsystems
    private final ArmSubsystem m_arm;
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final LauncherSubsystem m_launcher = new LauncherSubsystem();

    // The Driver subsystem
    // private final DriverSubsystem m_robotDrive = new LauncherSubsystem();

    // The driver's controllers
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_weaponController = new XboxController(OIConstants.kWeaponControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_arm = new ArmSubsystem(m_weaponController);
        // Configure the button bindings
        configureDriverButtons();
        configureWeaponButtons();

        // Configure default commands
        /*m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () ->
                    m_robotDrive.drive(
                        -GamepadUtils.squareInput(
                            m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -GamepadUtils.squareInput(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -GamepadUtils.squareInput(
                            m_driverController.getRightX(), OIConstants.kDriveDeadband),
                        true,
                        false),
                m_robotDrive));
        */
        // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
        m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

        // set the intake to stop (0 power) when no other command is running
        m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

        // configure the launcher to stop when no other command is running
        m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureWeaponButtons() {
        WeaponControllerProfiles.getDefaultProfile(m_weaponController, m_arm, m_intake, m_launcher);
        //WeaponControllerProfiles.GetEvansProfile(m_weaponController, m_arm, m_intake, m_launcher);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureDriverButtons() {
        // button to put swerve modules in an "x" configuration to hold position
        // new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        // .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*   public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    } */
}
