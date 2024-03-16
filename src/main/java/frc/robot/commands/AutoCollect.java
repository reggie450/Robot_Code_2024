package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCollect extends ParallelCommandGroup {
    IntakeSubsystem s_intake;
    public AutoCollect(Command traverse, IntakeSubsystem intake){
        super();
        s_intake = intake;
        addCommands(new AutoIntakeCollect(intake, true), traverse);
    }
}

