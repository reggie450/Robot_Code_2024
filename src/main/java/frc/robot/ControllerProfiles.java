package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class ControllerProfiles {
    int m_armup;
    int m_armdown;
    int m_intake;
    int m_shoot;
    int m_place;
    int m_climber;
    private ControllerProfiles()
    {
        InitEvan();
    }

    public void InitEvan() {
        m_armup = XboxController.Button.kLeftBumper.value;
        m_armdown = XboxController.Button.kLeftBumper.value;
    }
}