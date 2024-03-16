package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCollect extends Command {
    IntakeSubsystem s_intake;
    public AutoIntakeCollect(IntakeSubsystem intake){
        super();
        s_intake = intake;
        addRequirements(s_intake);
        //SmartDashboard.putBoolean("AutoIntakeCollect Running - Debug", true);
    }
    
    @Override
    public void initialize() {
        s_intake.running = true;
    }

    @Override
    public void execute() {
        s_intake.run(.3);
    }

    @Override
    public boolean isFinished() {
        return s_intake.getLimitSwitch() || s_intake.collected;
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.collected = true;
        s_intake.stop();
        s_intake.running = false;
        //SmartDashboard.putBoolean("AutoIntakeCollect Running - Debug", false);
    }
}

