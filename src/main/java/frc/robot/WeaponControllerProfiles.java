package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmGoToTarget;
import frc.robot.commands.Launch;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.Launch.ShotType;


public class WeaponControllerProfiles {
    public WeaponControllerProfiles(XboxController weapons, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher, ClimberSubsystem climber) {
        s_arm = arm;
        s_intake = intake;
        s_launcher = launcher;
        s_climber = climber;
        c_weapons = weapons;

        DefineCommands();

        GetProfile();
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
        intakeRunSimpler = new RunCommand(()->s_intake.intakeSimple());
        intakeStop = new InstantCommand(()->s_intake.stop(),s_intake);

        launcherStop = new InstantCommand(() -> s_launcher.stop(), s_launcher);
        launcherAmpShotCommand = new Launch(s_launcher,s_intake, ShotType.ampShot,false);
        launcherSpeakerShotCommand = new Launch(s_launcher, s_intake, ShotType.speakerShot, false);
        launcherOwenWilsonSucksCommand = new Launch(s_launcher, s_intake, ShotType.owenWilsonSucks, false);
        armResetEncoder = new InstantCommand(() -> s_arm.resetEncoder());
    }

    public void GetProfile() {
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
            .whileTrue(intakeRunSimpler).onFalse(intakeStop);    

        // new JoystickButton(c_weapons, XboxController.Button.kB.value)
        //     .onTrue(launcherAmpShotCommand);

        new JoystickButton(c_weapons, XboxController.Button.kX.value)
            .onTrue(new ArmGoToTarget(s_arm,.25));
            
        /* Launcher Controls */
        new POVButton(c_weapons, 270)
            .onTrue(launcherAmpShotCommand);

        new POVButton(c_weapons, 0)
            .onTrue(launcherSpeakerShotCommand);

        new POVButton(c_weapons, 90)
            .onTrue(launcherOwenWilsonSucksCommand);

        new JoystickButton(c_weapons, XboxController.Button.kStart.value)
            .onTrue(armResetEncoder);
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
    RunCommand intakeRunSimpler;
    InstantCommand intakeStop;

    InstantCommand launcherStop;
    Command launcherAmpShotCommand;
    Command launcherSpeakerShotCommand;
    Command launcherOwenWilsonSucksCommand;
    InstantCommand armResetEncoder;
    InstantCommand limeOn;
}