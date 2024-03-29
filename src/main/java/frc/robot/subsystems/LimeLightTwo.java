// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightTwo extends SubsystemBase {
  
  /** Creates a new LimeLightOne. */
    public final static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-two");


    private final NetworkTableEntry pipeline = table.getEntry("pipeline");
    private final NetworkTableEntry xOffset = table.getEntry("tx");
    private final NetworkTableEntry yOffset = table.getEntry("ty");

    private int currentPipeline = 0;


    @Override
    public void periodic(){
     // table.getEntry("ledMode").setNumber(3);
        //table.getEntry("camMode").setNumber(0);
    }

    /**
     * Toggles between two pipelines identical
     * other than one is 2x zoomed.
     */
    public void toggleZoom(){
        if(currentPipeline == 0)
            currentPipeline = 1;
        else if(currentPipeline == 1)
            currentPipeline = 0;

        setPipeline(currentPipeline);
    }

   
    /**
     * Calculates distance to target using techniques from:
     * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#fixed-angle-camera
     * @return distance to target
     */
    public double getDistanceFromTarget(){
        
        double targetOffsetAngle_Vertical = getYOffset();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0; 

        // distance from the target to the floor
        double goalHeightInches = 60.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        //return Constants.LL_SHOT_HEIGHT / Math.abs(Math.toRadians(getYOffset()));
        return distanceFromLimelightToGoalInches;
    } 

    public double getXOffset(){
        return xOffset.getDouble(0);
    }

    public double getYOffset(){
        return yOffset.getDouble(0);
    }

    public void setPipeline(int id){
        pipeline.setNumber(id);
    }

    public void LimeOff(){
        table.getEntry("ledMode").setNumber(1);
    }

    public void LimeOn(){
        table.getEntry("ledMode").setNumber(3);
    }
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
    public void FilterTags(){

    }


    public void CameraMode(){

        table.getEntry("camMode").setNumber(1);

    }

 
}

