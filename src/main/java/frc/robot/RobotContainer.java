package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
    private final JoystickButton robotCentricSwap = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    private boolean robotCentric = false;
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private static SendableChooser<Command> autoChooser;  

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_arm = new ArmSubsystem();
        
        autoChooser = new SendableChooser<Command>();
        //autoChooser.setDefaultOption(name: "Default High Cone", new HighCone(s_Swerve, m_crane, m_extender, m_grabber));
        //autoChooser.setDefaultOption(name: "High Cone / Back Out of Zone", new HighCone(s_Swerve, m_crane, m_extender, m_grabber));
        //autoChooser.addOption(name: "Lower Cone / Back Out", new LowCone(s_Swerve, m_crane, m_extender, m_grabber)); 
        //autoChooser.addOption(name: "Back Out", new exampleAuto(s_Swerve));
        //SmartDashboard.putData(key: "Auto mode", autoChooser); 

        // Configure the button bindings
        configureDriverButtons();
        configureWeaponButtons();

    

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -m_driverController.getRawAxis(translationAxis), 
                () -> -m_driverController.getRawAxis(strafeAxis), 
                () -> -m_driverController.getRawAxis(rotationAxis), 
                () -> robotCentricSwap.getAsBoolean() ? robotCentric : !robotCentric
            )
        );

        // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
        m_arm.setDefaultCommand(
            new RunCommand(
                () -> m_arm.runManual(m_weaponController.getRawAxis(1)), m_arm
                ));

        // set the intake to stop (0 power) when no other command is running
        m_intake.setDefaultCommand(new RunCommand(() -> m_intake.collectPayload(m_arm.getEncoderPosition()), m_intake));
    
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
        // completely swap robot centric field centric
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new Command() {    
              @Override
              public void execute() {
                robotCentric = !robotCentric;
              }});

        JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);


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
