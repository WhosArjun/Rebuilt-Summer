package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionReadings {
    private Pose3d pose; //Pose3d : X,Y,Z & Roll, Pitch, Yaw
    private double timestamp; //Stores when the image is captured
    private int tagCount;
    private double ambigiutiy;
    private String cameraName;
    public VisionReadings(Pose3d position, double timestamp){
        this.pose = position;
        this.timestamp = timestamp;
    }
    //Convert to Pose2d because most robots drive on a flat field (x,y,heading) ignores(z,roll, pitch)
    public Pose2d getPose2d(){
        return pose.toPose2d();
    }

    public double getTimestampSeconds(){
        return timestamp;
    }

    public Pose3d getPose3d(){
        return pose;
    }

    public int getTagCount(){
        return tagCount;
    }

    public double getAmbiguity(){
        return ambigiutiy;
    }

    public String getName(){
        return cameraName;
    } 
}
