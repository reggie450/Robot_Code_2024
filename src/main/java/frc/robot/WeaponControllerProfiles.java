package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
        /*   new Trigger(
                () ->
                    weaponController.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new RunCommand(() -> intake.setPower(Constants.Intake.kIntakePower), intake))
            .onFalse(intake.retract());
        //
        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(() -> intake.collectPayload(1.0,true)))
            .onFalse(new RunCommand(() -> intake.setPower(0)));
        // Run Feed Launcher
        new JoystickButton(weaponController, XboxController.Button.kA.value)
            .onTrue(intake.feedLauncher(launcher)); */


        // launcher controls (button to pre-spin the launcher and button to launch)
        new JoystickButton(weaponController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> launcher.runLauncher(), launcher));



        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .whileTrue(new RunCommand(() -> climber.climbUpLeft(), climber))
            .onFalse(new RunCommand(() -> climber.climbStopLeft(), climber));
    }

    public static void GetEvansProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher, ClimberSubsystem climber) {
        new Trigger( 
                () ->
                    weaponController.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new InstantCommand(() -> climber.climbUpLeft()))
            .onFalse(new InstantCommand(() -> climber.climbStopLeft(), climber));
         new Trigger( 
                () ->
                    weaponController.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new InstantCommand(() -> climber.climbUpRight()))
            .onFalse(new InstantCommand(() -> climber.climbStopRight(), climber));

        
        new JoystickButton(weaponController, XboxController.Button.kRightBumper.value)
            .whileTrue(new InstantCommand(() -> climber.climbDownRight(), climber))
            .onFalse(new InstantCommand(() -> climber.climbStopRight(), climber));

        new JoystickButton(weaponController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new InstantCommand(() -> climber.climbDownLeft(), climber))
            .onFalse(new InstantCommand(() -> climber.climbStopLeft(), climber));

        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .onTrue(new RunCommand(() -> launcher.ampShot(), launcher));

        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .whileFalse(new InstantCommand(() -> launcher.stopShooter(), launcher));

        new JoystickButton(weaponController, XboxController.Button.kX.value)
            .onTrue(new RunCommand(() -> launcher.speakerShot(), launcher))
            .whileFalse(new InstantCommand(() -> launcher.stopShooter(), launcher));


        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(()->IntakeSubsystem.intakeRun(.3)))
            .whileFalse(new InstantCommand(()->IntakeSubsystem.intakeStop()));

        // Set to Home Positions
        new JoystickButton(weaponController, XboxController.Button.kRightStick.value)
            .onTrue(new RunCommand(() -> arm.setTargetPosition(Constants.Arm.kHomePosition)));

        new POVButton(weaponController, 270)
            .onTrue(new RunCommand(() -> launcher.ampShot(), launcher));
        new POVButton(weaponController, 270)
            .onFalse(new RunCommand(() -> launcher.stopShooter(), launcher));

        new POVButton(weaponController, 0)
            .onTrue(new RunCommand(() -> launcher.speakerShot(), launcher));
        new POVButton(weaponController, 0).onFalse(new RunCommand(() -> launcher.stopShooter(), launcher));

        new POVButton(weaponController, 90)
            .onTrue(new RunCommand(()-> launcher.owenWilsonSucks(), launcher));
        new POVButton(weaponController, 90)
            .onFalse(new RunCommand(()-> launcher.stopShooter(), launcher));

    }

    public static void GetAliceProfile(XboxController weaponController, ArmSubsystem arm, IntakeSubsystem intake, LauncherSubsystem launcher, ClimberSubsystem climber) {
        new Trigger( 
                () ->
                    weaponController.getLeftTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new InstantCommand(() -> climber.climbUpLeft()))
            .onFalse(new InstantCommand(() -> climber.climbStopLeft(), climber));
        new Trigger( 
                () ->
                    weaponController.getRightTriggerAxis()
                        > Constants.OIConstants.kTriggerButtonThreshold)
            .whileTrue(new InstantCommand(() -> climber.climbUpRight()))
            .onFalse(new InstantCommand(() -> climber.climbStopRight(), climber));

        
        new JoystickButton(weaponController, XboxController.Button.kRightBumper.value)
            .whileTrue(new InstantCommand(() -> climber.climbDownRight(), climber))
            .onFalse(new InstantCommand(() -> climber.climbStopRight(), climber));

        new JoystickButton(weaponController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new InstantCommand(() -> climber.climbDownLeft(), climber))
            .onFalse(new InstantCommand(() -> climber.climbStopLeft(), climber));

        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .onTrue(new RunCommand(() -> launcher.ampShot(), launcher));

        new JoystickButton(weaponController, XboxController.Button.kB.value)
            .whileFalse(new InstantCommand(() -> launcher.stopShooter(), launcher));

        new JoystickButton(weaponController, XboxController.Button.kX.value)
            .onTrue(new RunCommand(() -> launcher.speakerShot(), launcher))
            .whileFalse(new InstantCommand(() -> launcher.stopShooter(), launcher));

        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileTrue(new RunCommand(()->IntakeSubsystem.intakeRun(.3)));

        new JoystickButton(weaponController, XboxController.Button.kY.value)
            .whileFalse(new InstantCommand(()->IntakeSubsystem.intakeStop()));

        // Set to Home Positions
        new JoystickButton(weaponController, XboxController.Button.kRightStick.value)
            .onTrue(new RunCommand(() -> arm.setTargetPosition(Constants.Arm.kHomePosition)));

        new POVButton(weaponController, 270)
            .onTrue(new RunCommand(() -> launcher.ampShot(), launcher));
        new POVButton(weaponController, 270)
            .onFalse(new RunCommand(() -> launcher.stopShooter(), launcher));

        new POVButton(weaponController, 0)
            .onTrue(new RunCommand(() -> launcher.speakerShot(), launcher));
        new POVButton(weaponController, 0)
            .onFalse(new RunCommand(() -> launcher.stopShooter(), launcher));

        new POVButton(weaponController, 90)
            .onTrue(new RunCommand(()-> launcher.owenWilsonSucks(), launcher));
        new POVButton(weaponController, 90)
            .onFalse(new RunCommand(()-> launcher.stopShooter(), launcher));

    }

}