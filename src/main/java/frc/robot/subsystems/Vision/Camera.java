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
    private final String name; //name of camera
    private final PhotonCamera cam; //models of the photon camera
    private final PhotonPoseEstimator poseEstimator; //position estimator of the cameras returned values

    //Constructor of the camera values 
    public Camera(String name, Transform3d kRobotToCam){
        this.name = name;
        this.cam = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(Constants.kTagLayout, kRobotToCam); //Creates new position estimator
    }

    //Estimates the pose by collecting the vision readings absorbed by the camera
    List<VisionReading> estimatePose(){
        List<PhotonPipelineResult> results = cam.getAllUnreadResults(); //Gets all the undread results of the camera to read
        List<VisionReading> readings = new ArrayList<>(); //Creates a new array list with all the results
        for(PhotonPipelineResult res : results){ //Enhanced for loop, for all the readings in the results...
            Optional<EstimatedRobotPose> x = Optional.empty();

            if(res.getMultiTagResult().isPresent()){ //If the reading is good enough, and if the cmaeras reading is real
                x = poseEstimator.estimateCoprocMultiTagPose(res); //Apply a pose estimator to get the position of the robot
            }
            if(x.isPresent()){
                readings.add(new VisionReading(x.get().estimatedPose, res.getTimestampSeconds()));
            }
        }
        return readings; //Returns all the estimated positions that keep updating
    }

    public boolean isConnected(){
        return cam.isConnected();
    }
    public String toString(){
        return "{Name: " + name + ", isActive: " + isConnected() + "}";
    }

}

//Private class that classifies what a vision reading is
class VisionReading{
    private Pose3d pose; //Positon of the bot
    private double timestamp; //timeStamp of the match

    public VisionReading(Pose3d pose, double timestamp){
        this.pose = pose;
        this.timestamp = timestamp;
    }

    //Turns the position into a Pose2d, 2d plane (x and y)
    public Pose2d getPose2d(){
        return pose.toPose2d();
    }

    public double getTimeStampSeconds(){
        return timestamp;
    }
}
