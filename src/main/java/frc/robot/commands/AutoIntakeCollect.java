package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCollect extends Command {
    IntakeSubsystem s_intake;
    Timer m_timer = new Timer();
    boolean timeout = false;
    boolean m_withTimeout;
    public AutoIntakeCollect(IntakeSubsystem intake, boolean withTimeout){
        super();
        s_intake = intake;
        m_withTimeout = withTimeout;
        addRequirements(s_intake);
        //SmartDashboard.putBoolean("AutoIntakeCollect Running - Debug", true);
        if (m_withTimeout)
          m_timer.start();
    }
    
    @Override
    public void initialize() {
        s_intake.running = true;
        if (m_withTimeout){
            m_timer.reset();
        }
        timeout = false;
    }

    @Override
    public void execute() {
        s_intake.run(.3);
    }

    @Override
    public boolean isFinished() {
        boolean noteCollected = s_intake.getLimitSwitch() || s_intake.collected;
        if (!noteCollected && m_withTimeout && m_timer.get() > 3)
            timeout = true;
        return noteCollected || timeout;
    }

    @Override
    public void end(boolean interrupted) {
        if (timeout){
            s_intake.collected = false;
        }
        else{
            s_intake.collected = true;
        }
        s_intake.stop();
        s_intake.running = false;
        //SmartDashboard.putBoolean("AutoIntakeCollect Running - Debug", false);
    }
}

