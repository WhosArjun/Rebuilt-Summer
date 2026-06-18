package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionMeasurement {
    public final Pose3d pose; //Robot pose estimated from camera (3d space)
    public final double timeStamp; //When this measurement was taken (latency correction)
    public final double confidence; //How much we trust this measurement 
    public final String cameraName; //Which camera produced the measurement
    public final int tagId;

    public VisionMeasurement(Pose3d pose, double timestamp, double confidence, String name, int tagId){
        this.pose = pose;
        this.timeStamp = timestamp;
        this.confidence = confidence;
        this.cameraName = name;
        this.tagId = tagId;
    }

    public int getTagId(){
        return tagId;
    }
}
