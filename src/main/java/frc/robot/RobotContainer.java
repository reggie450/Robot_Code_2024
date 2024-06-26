package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
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
    private final ArmSubsystem s_arm = new ArmSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final LauncherSubsystem s_launcher = new LauncherSubsystem();
    private final ClimberSubsystem s_climber = new ClimberSubsystem();

    private final LimeLightTwo s_limeLightTwo = new LimeLightTwo();

    // The driver's controllers
    Joystick j_driver = new Joystick(OIConstants.kDriverControllerPort);
    XboxController c_weapon = new XboxController(OIConstants.kWeaponControllerPort);

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(j_driver, 4);
    private final JoystickButton zeroHeading = new JoystickButton(j_driver, 3);
    private final JoystickButton limeOn = new JoystickButton(j_driver, 11);
    private final JoystickButton slow_mode = new JoystickButton(j_driver, 1);

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
        autoChooser.addOption("Shoot Only", new ShootOnly(s_launcher, s_intake, s_arm));
        //autoChooser.addOption("Dance Test", new Dance(s_Swerve, s_launcher, s_intake, s_arm));
       // autoChooser.addOption("Dance Test1", new Dance1(s_Swerve, s_launcher, s_intake, s_arm));
        autoChooser.addOption("Aim Test", new AimTest(s_launcher, s_intake, s_arm));
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Configure the button bindings
        configureDriverButtons();
        weaponProfile = new WeaponControllerProfiles(c_weapon, s_arm, s_intake, s_launcher, s_climber);
        //weaponProfile = new WeaponControllerProfiles(WeaponProfile.Evan, c_weapon, s_arm, s_intake, s_launcher, s_climber);
        j_driver.setXChannel(1);
        j_driver.setYChannel(0);
        j_driver.setTwistChannel(2);
        j_driver.setThrottleChannel(3);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> j_driver.getX(),
                () -> j_driver.getY(), //j_driver.getRawAxis(3),
                () -> -j_driver.getTwist() /2.0,// -j_driver.getRawAxis(2) / 2,
                () -> !robotCentric.getAsBoolean(),
                () -> slow_mode.getAsBoolean()
                ));

        s_limeLightTwo.CameraMode();
        // set the arm subsystem to run the "runAutomatic" function continuously when no
        // other command is running
        s_arm.setDefaultCommand(
                new RunCommand(
                        () -> s_arm.runManual(c_weapon.getRawAxis(1), c_weapon.getRawAxis(5)), s_arm));

        // configure the launcher to stop when no other command is running
        s_launcher.setDefaultCommand(new RunCommand(() -> s_launcher.stop(), s_launcher));
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
        //new JoystickButton(j_driver, 7).onTrue(new InstantCommand(() -> s_launcher.PlaySong()));
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
        //if (!s_intake.collected && !s_intake.running){
            //IntakeCollect intakeCommand = new IntakeCollect(s_intake,s_arm, faWlse);
            //intakeCommand.schedule();
        //}
        
    }
}
