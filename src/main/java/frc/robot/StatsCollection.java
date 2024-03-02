package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StatsCollection {
    private static Boolean CollectionOn = false;
    private Timer periodicTimer;
    private Timer methodTimer;
    private double[] periodic = {0.0,0.0,0.0,0.0,0.0};        
    private int nextReadingIndex = 0;

    //private Dictionary<String,List<double>> method = {"MethodName", {0.0,0.0,0.0,0.0,0.0}};        
    //private int nextmethodIndex = 0;
    private String m_label = "";

    public StatsCollection(String label) {
        if (CollectionOn){
            periodicTimer = new Timer();   
            periodicTimer.start();
            m_label = label;
        }
    }

    public void Periodic() {
        if (CollectionOn){
            periodic[nextReadingIndex] = periodicTimer.get();
            if (nextReadingIndex + 1 == periodic.length) nextReadingIndex = 0;
            else ++nextReadingIndex;
            periodicTimer.reset();
            double averageSeconds = (periodic[0] + periodic[1]+ periodic[2] + periodic[3] + periodic[4]) / 5;
            SmartDashboard.putNumber(m_label, averageSeconds);
        }
    }

    public void MethodStart(String methodName) {
        if (CollectionOn){
            if (methodTimer == null) methodTimer = new Timer();
            else methodTimer.reset();
            methodTimer.start();
        }
    }

    public void MethodEnd(String methodName) {
        if (CollectionOn){
            double value = methodTimer.get();
            SmartDashboard.putNumber(m_label + ":"+ methodName, value);
            methodTimer.stop();
        }
    }
}
