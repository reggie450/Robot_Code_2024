package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public static class WeaponControllerProfiles {
    public void getDefaultProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher) {
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

    public void GetEvansProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher) {

    }

}