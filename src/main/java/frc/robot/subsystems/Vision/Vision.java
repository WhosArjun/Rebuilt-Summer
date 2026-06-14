package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public Camera shutter;  
    public Camera ardu;
    public boolean visionOn = true;
    BiConsumer<Pose2d, Double> updateDrivetrain; //Biconsumer is a function that takes two inputs and returns nothing
    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(0.67, 0.67, 0.67, new Rotation3d(0,Math.toRadians(67),Math.toRadians(67))));
        ardu = new Camera("Arducam623", new Transform3d(0.67, -0.67, 0.67, new Rotation3d(0,Math.toRadians(67),Math.toRadians(67))));
        this.updateDrivetrain = updateDrivetrain;
    }

    public void periodic(){
        SmartDashboard.putBoolean("Vision", visionOn);
        if(visionOn){
            List<VisionReadings> VisionReadings = new ArrayList<>(); //stores all readings from both cameras
            for(VisionReadings reading : shutter.estimatePose()){ //Any new shuttercamera readings, if so add to list
                VisionReadings.add(reading);
            }
            for(VisionReadings reading : ardu.estimatePose()){ //Any new arducam readings, is so add to list
                VisionReadings.add(reading);
            }
            for(VisionReadings reading : VisionReadings){
                updateDrivetrain.accept(reading.getPose2d(), reading.getTimestampSeconds()); //Sending readings to drivetrain, passing both pose2d and timestamp into the call back
            }
        }
    }
}
