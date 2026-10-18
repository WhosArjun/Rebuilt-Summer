package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionReal extends VisionIO{
    public Camera shutter; //Shutter camera
    public Camera ardu;     //Ardu Camera
    public boolean visionOn = true; //If the vision is on the camera
    BiConsumer<Pose2d, Double> updateDrivetrain; //A biconsumer datastructure that holds the position, and the double value of the robot.

    public VisionReal(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(0.263456, 0.2980182, 0.512445, new Rotation3d(0, Math.toRadians(-18.3), Math.toRadians(11)))); //Initializes the camera position on the bot
        ardu = new Camera("Arducam623", new Transform3d(0.263092, -0.300228, 0.5041392, new Rotation3d(0, Math.toRadians(-9.2), Math.toRadians(-10)))); //Initializes the camera position on the bot
        this.updateDrivetrain = updateDrivetrain;
    }
    //Methods that updates periodically every 20 milliseconds
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("vision", visionOn); //For debugging purposes to see if vision of the camera si even on

        //If the vision is not on return outside the method
        if(!visionOn){
            return;
        } 
        List<VisionReading> readings = new ArrayList<>(); //If the vision is on it creates a vision reading in an arrayList

        //Stores the visionreadings of the shutter camera
         for(VisionReading reading : shutter.estimatePose()){
             readings.add(reading);
             SmartDashboard.putNumber("Shutter pose", reading.getPose2d().getX());
             SmartDashboard.putNumber("Shutter pose y", reading.getPose2d().getY());
         }
        
         //Stores the visionreadings of the ardu camera
        for(VisionReading reading : ardu.estimatePose()){
             readings.add(reading);
             SmartDashboard.putNumber("Arducam pose", reading.getPose2d().getX());
             SmartDashboard.putNumber("Arducam pose y", reading.getPose2d().getY());
         }
        
         //Updates the drivetrain with these readings from the arrylist, and updates the position and the timestamp of when and where the robot is on the field.
         for(VisionReading reading : readings){
             updateDrivetrain.accept(reading.getPose2d(), reading.getTimeStampSeconds());
         }
    }
}
