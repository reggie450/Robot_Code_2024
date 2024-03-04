package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCollect extends Command {
    ArmSubsystem s_arm;
    IntakeSubsystem s_intake;
    boolean m_armBased;
    public IntakeCollect(IntakeSubsystem intake, ArmSubsystem arm, boolean armBased){
        super();
        s_arm = arm;
        s_intake = intake;
        m_armBased = armBased;
        addRequirements(s_intake, s_arm);
    }
    @Override
    public void initialize() {
        s_intake.running = true;
    }

    @Override
    public void execute() {
        if (!s_intake.collected && (!m_armBased || s_arm.getEncoderPosition() >= Constants.Arm.kIntakePosition))
            s_intake.run(-.4);
        else {
            s_intake.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return s_intake.collected || s_intake.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.stop();
        s_intake.collected = true;
        s_intake.running = false;
    }
}
