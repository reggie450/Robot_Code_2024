package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StatsCollection {
    private static Boolean CollectionOn = true;
    private Timer periodicTimer;
    private Timer methodTimer;
    private double[] periodic = {0.0,0.0,0.0,0.0,0.0};        
    private int nextReadingIndex = 0;

    private double[] method = {0.0,0.0,0.0,0.0,0.0};        
    private int nextmethodIndex = 0;

    public StatsCollection() {
        if (CollectionOn){
            periodicTimer = new Timer();   
            periodicTimer.start();
        }
    }

    public void PeriodicStart() {
        if (CollectionOn){
            periodic[nextReadingIndex] = periodicTimer.get();
            if (nextReadingIndex + 1 == periodic.length) nextReadingIndex = 0;
            else ++nextReadingIndex;
            periodicTimer.reset();
        }
    }

    public void MethodStart() {
        if (CollectionOn){
            if (methodTimer == null) methodTimer = new Timer();
            else methodTimer.reset();
            methodTimer.start();
        }
    }

    public void MethodEnd(String label) {
        if (CollectionOn){
            double value = methodTimer.get();
            SmartDashboard.putNumber(label, value);
            methodTimer.stop();
        }
    }
}
