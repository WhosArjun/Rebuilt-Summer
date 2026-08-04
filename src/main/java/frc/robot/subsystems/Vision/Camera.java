package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class Camera {
    private final String name;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator;

    public Camera(String name, Transform3d kRobotToCam){
        this.name = name;
        this.cam = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(Constants.kTagLayout, kRobotToCam);
    }

    List<VisionReading> estimatePose(){
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        List<VisionReading> readings = new ArrayList<>();
        for(PhotonPipelineResult res : results){
            Optional<EstimatedRobotPose> x = Optional.empty();

            if(res.getMultiTagResult().isPresent()){
                x = poseEstimator.estimateCoprocMultiTagPose(res);
            }
            if(x.isPresent()){
                readings.add(new VisionReading(x.get().estimatedPose, res.getTimestampSeconds()));
            }
        }
        return readings;
    }

    public boolean isConnected(){
        return cam.isConnected();
    }
    public String toString(){
        return "{Name: " + name + ", isActive: " + isConnected() + "}";
    }
}

class VisionReading{
    private Pose3d pose;
    private double timestamp;

    public VisionReading(Pose3d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public Pose2d getPose2d(){
        return pose.toPose2d();
    }

    public double getTimeStampSeconds(){
        return timestamp;
    }
}
