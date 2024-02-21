package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class WeaponControllerProfiles {
    public static void getDefaultProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher, ClimberSubsystem climber) {
        // set up arm preset positions
        // Set to Scoring Position
        new JoystickButton(weaponController, XboxController.Button.kLeftBumper.value) 
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kScoringPosition)));
        // Set to Intake Position
        new Trigger( 
                () ->
                    weaponController.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kIntakePosition)));
        // Set to Home Positions
        new JoystickButton(weaponController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kHomePosition)));


        // intake controls (run while button is held down, run retract command once when the button is released)
        new Trigger(
                () ->
                    weaponController.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new RunCommand(() -> intake.setPower(Constants.Intake.kIntakePower), intake))
            .onFalse(intake.retract());
        //
        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(() -> intake.setPower(-1.0)));
        // Run Feed Launcher
        new JoystickButton(weaponController, XboxController.Button.kA.value)
            .onTrue(intake.feedLauncher(launcher));


        // launcher controls (button to pre-spin the launcher and button to launch)
        new JoystickButton(weaponController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> launcher.runLauncher(), launcher));



        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .whileTrue(new RunCommand(() -> climber.climb(), climber))
            .onFalse(new RunCommand(() -> climber.stop(), climber));
    }

    public void GetEvansProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher) {
        // set up arm preset positions
        // Set to Scoring Position
        new JoystickButton(weaponController, XboxController.Button.kLeftBumper.value) 
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kScoringPosition)));
        // Set to Intake Position
        new Trigger( 
                () ->
                    weaponController.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kIntakePosition)));
        // Set to Home Positions
        new JoystickButton(weaponController, XboxController.Button.kStart.value)
            .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.Arm.kHomePosition)));


        // intake controls (run while button is held down, run retract command once when the button is released)
        new Trigger(
                () ->
                    weaponController.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new RunCommand(() -> intake.setPower(Constants.Intake.kIntakePower), intake))
            .onFalse(intake.retract());
        //
        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(() -> intake.setPower(-1.0)));
        // Run Feed Launcher
        new JoystickButton(weaponController, XboxController.Button.kA.value)
            .onTrue(intake.feedLauncher(launcher));


        // launcher controls (button to pre-spin the launcher and button to launch)
        new JoystickButton(weaponController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> launcher.runLauncher(), launcher));



    }

}