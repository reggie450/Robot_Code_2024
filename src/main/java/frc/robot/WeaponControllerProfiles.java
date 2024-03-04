package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSubsystem.ShotType;


public class WeaponControllerProfiles {
    public WeaponControllerProfiles(WeaponProfile profile, XboxController weapons, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher, ClimberSubsystem climber) {
        s_arm = arm;
        s_intake = intake;
        s_launcher = launcher;
        s_climber = climber;
        c_weapons = weapons;

        DefineCommands();

        if (profile == WeaponProfile.Alice) {
            GetAliceProfile();
        }
        else if (profile == WeaponProfile.Evan) {
            GetEvansProfile();
        }
        else {
            GetDefaultProfile();
        }
    }

    public void DefineCommands(){
        // ClimberCommands
        climberLeftUp = new InstantCommand(() -> s_climber.LeftUp(),s_climber);
        climberLeftDown = new InstantCommand(() -> s_climber.LeftDown(),s_climber);
        climberLeftStop = new InstantCommand(() -> s_climber.LeftStop(),s_climber);
        climberRightUp = new InstantCommand(() -> s_climber.RightUp(),s_climber);
        climberRightDown = new InstantCommand(() -> s_climber.RightDown(),s_climber);
        climberRightStop = new InstantCommand(() -> s_climber.RightStop(),s_climber);

        intakeRun = new InstantCommand(()->s_intake.run(.3),s_intake);
        intakeStop = new InstantCommand(()->s_intake.stop(),s_intake);
        intakebackup = new InstantCommand(()->s_intake.backup(),s_intake);
        intakeStopRetract = s_intake.stopRetract();

        launcherAmpShot = new InstantCommand(() -> s_launcher.primeShot(ShotType.ampShot), s_launcher);
        launcherSpeakerShot = new InstantCommand(() -> s_launcher.primeShot(ShotType.speakerShot), s_launcher);
        launcherOwenWilsonSucks = new InstantCommand(() -> s_launcher.primeShot(ShotType.owenWilsonSucks), s_launcher);
        launcherStop = new InstantCommand(() -> s_launcher.stopShooter(), s_launcher);
        launcherAmpShotCommand = s_launcher.ShotIntake(s_intake, ShotType.ampShot);
        launcherSpeakerShotCommand = s_launcher.ShotIntake(s_intake, ShotType.speakerShot);
        launcherOwenWilsonSucksCommand = s_launcher.ShotIntake(s_intake, ShotType.owenWilsonSucks);
    }

    public void GetEvansProfile() {
        /* Climber Controls */
        new Trigger( 
                () -> c_weapons.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberLeftUp).onFalse(climberLeftStop);

        new JoystickButton(c_weapons, XboxController.Button.kLeftBumper.value)
            .onTrue(climberLeftDown).onFalse(climberLeftStop);

         new Trigger( 
                () -> c_weapons.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberRightUp).onFalse(climberRightStop);

        new JoystickButton(c_weapons, XboxController.Button.kRightBumper.value)
            .onTrue(climberRightDown).onFalse(climberRightStop);

        /* Intake Controls */
        new JoystickButton(c_weapons, XboxController.Button.kY.value)
            .onTrue(intakeRun).onFalse(intakeStop);

        /* Launcher Controls */
        new JoystickButton(c_weapons, XboxController.Button.kB.value)
            .onTrue(launcherAmpShot).onFalse(launcherStop);

        new JoystickButton(c_weapons, XboxController.Button.kX.value)
            .onTrue(launcherSpeakerShot).onFalse(launcherStop);

        new POVButton(c_weapons, 270)
            .onTrue(launcherAmpShot).onFalse(launcherStop);

        new POVButton(c_weapons, 0)
            .onTrue(launcherSpeakerShot).onFalse(launcherStop);

        new POVButton(c_weapons, 90)
            .onTrue(launcherOwenWilsonSucks).onFalse(launcherStop);
    }

    public void GetAliceProfile() {
        /* Climber Controls */
        new Trigger( 
                () -> c_weapons.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberLeftUp).onFalse(climberLeftStop);

        new JoystickButton(c_weapons, XboxController.Button.kLeftBumper.value)
            .onTrue(climberLeftDown).onFalse(climberLeftStop);

        new Trigger( 
                () -> c_weapons.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberRightUp).onFalse(climberRightStop);

        new JoystickButton(c_weapons, XboxController.Button.kRightBumper.value)
            .onTrue(climberRightDown).onFalse(climberRightStop);

        /* Intake Controls */
        new JoystickButton(c_weapons, XboxController.Button.kY.value)
            .onTrue(intakeRun).onFalse(intakeStopRetract);

        new JoystickButton(c_weapons, XboxController.Button.kB.value)
            .onTrue(intakebackup).onFalse(intakeStop);

        // new JoystickButton(c_weapons, XboxController.Button.kB.value)
        //     .onTrue(launcherAmpShot).onFalse(launcherStop);

        // new JoystickButton(c_weapons, XboxController.Button.kX.value)
        //     .onTrue(launcherSpeakerShot).onFalse(launcherStop);
            
        /* Launcher Controls */
        new POVButton(c_weapons, 270)
            .onTrue(launcherAmpShotCommand);

        new POVButton(c_weapons, 0)
            .onTrue(launcherSpeakerShotCommand);

        new POVButton(c_weapons, 90)
            .onTrue(launcherOwenWilsonSucksCommand);
    }

    public void GetDefaultProfile() {
        /* Climber Controls */
        new Trigger( 
                () -> c_weapons.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberLeftUp).onFalse(climberLeftStop);

        new JoystickButton(c_weapons, XboxController.Button.kLeftBumper.value)
            .onTrue(climberLeftDown).onFalse(climberLeftStop);

         new Trigger( 
                () -> c_weapons.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(climberRightUp).onFalse(climberRightStop);

        new JoystickButton(c_weapons, XboxController.Button.kRightBumper.value)
            .onTrue(climberRightDown).onFalse(climberRightStop);

        /* Intake Controls */
        new JoystickButton(c_weapons, XboxController.Button.kY.value)
            .onTrue(intakeRun).onFalse(intakeStop);

        /* Launcher Controls */
        new JoystickButton(c_weapons, XboxController.Button.kB.value)
            .onTrue(launcherAmpShot).onFalse(launcherStop);

        new JoystickButton(c_weapons, XboxController.Button.kX.value)
            .onTrue(launcherSpeakerShot).onFalse(launcherStop);

        new POVButton(c_weapons, 270)
            .onTrue(launcherAmpShot).onFalse(launcherStop);

        new POVButton(c_weapons, 0)
            .onTrue(launcherSpeakerShot).onFalse(launcherStop);

        new POVButton(c_weapons, 90)
            .onTrue(launcherOwenWilsonSucks).onFalse(launcherStop);
    }

    public static enum WeaponProfile {
        Alice,
        Evan,
        Default; 
    }

    ArmSubsystem s_arm;
    IntakeSubsystem s_intake;
    LauncherSubsystem s_launcher;
    ClimberSubsystem s_climber;

    XboxController c_weapons;
    InstantCommand climberLeftUp;
    InstantCommand climberLeftDown;
    InstantCommand climberLeftStop;
    InstantCommand climberRightUp;
    InstantCommand climberRightDown;
    InstantCommand climberRightStop;

    InstantCommand intakeRun;
    InstantCommand intakeStop;
    InstantCommand intakebackup;
    Command intakeStopRetract;

    InstantCommand launcherAmpShot;
    InstantCommand launcherSpeakerShot;
    InstantCommand launcherOwenWilsonSucks;
    InstantCommand launcherStop;
    Command launcherAmpShotCommand;
    Command launcherSpeakerShotCommand;
    Command launcherOwenWilsonSucksCommand;

    InstantCommand limeOn;
}