package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Drivetrain;

public class VisionSim extends VisionIO{
    public VisionSystemSim visionSim = new VisionSystemSim("main");
    public Drivetrain drivetrain;
    // public TargetModel targetModel = TargetModel.kAprilTag36h11;
    // public Pose3d hubTargetPose = new Pose3d(11.901, 4.021, 1.124, new Rotation3d());
    // public VisionTargetSim visionTarget = new VisionTargetSim(hubTargetPose, targetModel);

    SimCameraProperties arducamProp = new SimCameraProperties();
    PhotonCameraSim arducam;

    public VisionSim(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark));

        arducamProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        arducamProp.setCalibError(0.25, 0.08);
        arducamProp.setFPS(60);
        arducamProp.setAvgLatencyMs(50);
        arducamProp.setLatencyStdDevMs(10);

        PhotonCamera real = new PhotonCamera("arducam");
        arducam = new PhotonCameraSim(real, arducamProp);

        Transform3d robotToCamera = new Transform3d(0.2630932, -0.300228, 0.5041392, new Rotation3d(0,Math.toRadians(-9.2),Math.toRadians(-10)));

        visionSim.addCamera(arducam, robotToCamera);


        
    }
    
    @Override
    public void periodic(){
        visionSim.update(drivetrain.swerveDrive.getPose());
        visionSim.getDebugField();
    }
}
