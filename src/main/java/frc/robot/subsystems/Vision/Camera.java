package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class Camera {
    private final String name;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator; //Converts apriltag observations into robot positions

    public Camera(String name, Transform3d robotToCam){ //robotToCam : Where camera is mounted relative to robot center
        this.name = name;
        this.cam = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(Constants.kTagLayout, robotToCam);
    }

    List<VisionReadings> estimatePose(){
        List<PhotonPipelineResult> results = cam.getAllUnreadResults(); //Gets all the camera readings that haven't been processed yet (Detected tags, ambigiutiy, and timestamp)
        List<VisionReadings> readings = new ArrayList<>(); //Created output list
        for(PhotonPipelineResult i : results){ //Loops through the results arraylist
            if(i.getMultiTagResult().isPresent()){ //Checks whether PhotonVision solved a MultiTag Pose
                                                    //Multitag means 2+ tags are visible
                Optional<EstimatedRobotPose> j = poseEstimator.estimateCoprocMultiTagPose(i); //Based on the observations, where is the robot?
                if(j.isPresent()){ //Only use succesful solutions
                    readings.add(new VisionReadings(j.get().estimatedPose, i.getTimestampSeconds())); //Stores robotpose and timestamp
                }
            }
            
        }
        return readings;
    }

    public boolean isCameraConnected(){
        boolean cameraConnected = cam.isConnected();
        String connected = "Name : " + name + ", isActive : " + cameraConnected; 
        System.out.println(connected);
        return cameraConnected;
    }

    
}
