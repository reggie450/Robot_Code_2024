package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.WeaponControllerProfiles.WeaponProfile;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/*
* This Class contains commands and button maps which call methods in the subsystems
 */
public class RobotContainer {
    // The Weapon subsystems
    private final ArmSubsystem s_arm = new ArmSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final LauncherSubsystem s_launcher = new LauncherSubsystem();
    private final ClimberSubsystem s_climber = new ClimberSubsystem();

    private final LimeLightTwo s_limeLightTwo = new LimeLightTwo();

    // The driver's controllers
    XboxController c_driver = new XboxController(OIConstants.kDriverControllerPort);
    XboxController c_weapon = new XboxController(OIConstants.kWeaponControllerPort);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(c_driver, XboxController.Button.kA.value);
    private final JoystickButton zeroHeading = new JoystickButton(c_driver, XboxController.Button.kY.value);
    private final JoystickButton limeOn = new JoystickButton(c_driver, XboxController.Button.kX.value);
    private final JoystickButton slow_mode = new JoystickButton(c_driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slow_mode2 = new JoystickButton(c_driver, XboxController.Button.kRightBumper.value);

    // private final JoystickButton robotCentricSwap = new
    // JoystickButton(c_driver, XboxController.Button.kLeftBumper.value);
    // private boolean robotCentric = true;
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private static SendableChooser<Command> autoChooser;

    private WeaponControllerProfiles weaponProfile;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Speaker, Drive Out", new SpeakerCenter(s_Swerve, s_launcher, s_intake, s_arm));
        autoChooser.addOption("Speaker Right, Drive Out",new SpeakerRight(s_Swerve, s_launcher, s_intake, s_arm));
        autoChooser.addOption("Speaker Left, Drive Out", new SpeakerLeft(s_Swerve, s_launcher, s_intake, s_arm));
        autoChooser.addOption("Shoot Only", new ShootOnly(s_Swerve, s_launcher, s_intake, s_arm));
        autoChooser.addOption("Dance", new Dance(s_Swerve, s_launcher, s_intake, s_arm));
        // autoChooser.setDefaultOption(name: "High Cone / Back Out of Zone", new
        // HighCone(s_Swerve, m_crane, m_extender, m_grabber));
        // autoChooser.addOption(name: "Lower Cone / Back Out", new LowCone(s_Swerve,
        // m_crane, m_extender, m_grabber));
        // autoChooser.addOption(name: "Back Out", new exampleAuto(s_Swerve));
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Configure the button bindings
        configureDriverButtons();
        weaponProfile = new WeaponControllerProfiles(WeaponProfile.Alice, c_weapon, s_arm, s_intake, s_launcher, s_climber);
        //weaponProfile = new WeaponControllerProfiles(WeaponProfile.Evan, c_weapon, s_arm, s_intake, s_launcher, s_climber);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -c_driver.getRawAxis(translationAxis),
                () -> -c_driver.getRawAxis(strafeAxis),
                () -> -c_driver.getRawAxis(rotationAxis) / 2,
                () -> !robotCentric.getAsBoolean(),
                () -> slow_mode.getAsBoolean()||slow_mode2.getAsBoolean() || c_driver.getRightTriggerAxis() > .5 || c_driver.getLeftTriggerAxis() > .5
                ));

        s_limeLightTwo.CameraMode();
        // set the arm subsystem to run the "runAutomatic" function continuously when no
        // other command is running
        s_arm.setDefaultCommand(
                new RunCommand(
                        () -> s_arm.runManual(c_weapon.getRawAxis(1)), s_arm));

        // configure the launcher to stop when no other command is running
        s_launcher.setDefaultCommand(new RunCommand(() -> s_launcher.stopLauncher(), s_launcher));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureDriverButtons() {
        zeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        limeOn.onTrue(new InstantCommand(() -> s_limeLightTwo.CameraMode()));
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public void teleopPeriodic() { // This method will be called once per scheduler run
        if (!s_intake.collected && !s_intake.running){
            //IntakeCollect intakeCommand = new IntakeCollect(s_intake,s_arm, false);
            //intakeCommand.schedule();
        }
    }
}
