package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/*
* This Class contains commands and button maps which call methods in the subsystems
 */
public class RobotContainer {
    // The Weapon subsystems
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final LauncherSubsystem m_launcher = new LauncherSubsystem();
    private final ClimberSubsystem m_climber = new ClimberSubsystem();

    private final LimeLightTwo m_limeLightTwo = new LimeLightTwo();

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
    private final JoystickButton robotCentric = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    private final JoystickButton zeroHeading = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton limeOn = new JoystickButton(m_driverController, XboxController.Button.kX.value);

    // private final JoystickButton robotCentricSwap = new
    // JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    // private boolean robotCentric = true;
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private static SendableChooser<Command> autoChooser;  

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Speaker, Drive Out", new Speaker(s_Swerve, m_launcher, m_intake, m_arm));
        // autoChooser.setDefaultOption(name: "High Cone / Back Out of Zone", new
        // HighCone(s_Swerve, m_crane, m_extender, m_grabber));
        // autoChooser.addOption(name: "Lower Cone / Back Out", new LowCone(s_Swerve,
        // m_crane, m_extender, m_grabber));
        // autoChooser.addOption(name: "Back Out", new exampleAuto(s_Swerve));
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Configure the button bindings
        configureDriverButtons();
        configureWeaponButtons();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -m_driverController.getRawAxis(translationAxis),
                () -> -m_driverController.getRawAxis(strafeAxis),
                () -> -m_driverController.getRawAxis(rotationAxis) / 2.5,
                () -> robotCentric.getAsBoolean()));

         m_limeLightTwo.CameraMode();
        // set the arm subsystem to run the "runAutomatic" function continuously when no
        // other command is running
        m_arm.setDefaultCommand(
                new RunCommand(
                        () -> m_arm.runManual(m_weaponController.getRawAxis(1)), m_arm));

        // configure the launcher to stop when no other command is running
        m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or
     * one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a {@link JoystickButton}.
     */
    private void configureWeaponButtons() {
        // WeaponControllerProfiles.getDefaultProfile(m_weaponController, m_arm, m_intake, m_launcher, m_climber);
        // WeaponControllerProfiles.GetEvansProfile(m_weaponController, m_arm, m_intake, m_launcher, m_climber);
        // limeOn.onTrue(new InstantCommand(() -> m_limeLightTwo.CameraMode()));
        WeaponControllerProfiles.GetAliceProfile(m_weaponController, m_arm, m_intake, m_launcher, m_climber);
        limeOn.onTrue(new InstantCommand(() -> m_limeLightTwo.CameraMode()));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or
     * one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a {@link JoystickButton}.
     */
    private void configureDriverButtons() {
        zeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
