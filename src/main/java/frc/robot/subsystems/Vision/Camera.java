package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class Camera {
    private final String name;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator estimator;

    public Camera(String name, Transform3d robotToCam){
        this.name = name;
        cam = new PhotonCamera(name);
        estimator = new PhotonPoseEstimator(Constants.kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public List<VisionMeasurement> estimatPose(){
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        List<VisionMeasurement> output = new ArrayList<>();

        Logger.recordOutput("Vision" + name + "frames", results.size());
        
        for(PhotonPipelineResult r : results){
            Optional<EstimatedRobotPose> estimate = estimator.update(r);
            if(estimate.isEmpty()){
                continue;
            }
            var pose = estimate.get().estimatedPose;
            int tagCount = r.targets.size();
            double ambiguity = (r.getBestTarget() != null)?r.getBestTarget().getPoseAmbiguity() : 1.0;
            double confidence = tagCount/(1.0+ambiguity);

            Logger.recordOutput("Vision" + name + "Pose", pose.toPose2d());
            Logger.recordOutput("Vision" + name + "Confidence", confidence);

            output.add(new VisionMeasurement(pose, r.getTimestampSeconds(), confidence, name));

        }

        return output;
    }
}
