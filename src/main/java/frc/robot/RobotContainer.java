package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
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
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_arm = new ArmSubsystem(m_weaponController);
        // Configure the button bindings
        configureDriverButtons();
        configureWeaponButtons();


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -m_driverController.getRawAxis(translationAxis), 
                () -> -m_driverController.getRawAxis(strafeAxis), 
                () -> -m_driverController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

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
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
