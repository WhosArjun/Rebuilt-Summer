package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private final Camera shutter;
    private final Camera ardu;
    private final SwerveDrivePoseEstimator poseEstimator; //Fuses odometry + vision

    public Vision(SwerveDrivePoseEstimator poseEstimator){
        this.poseEstimator = poseEstimator;
        shutter = new Camera("Shutter623", new Transform3d(0.67,0.67,0.67, new Rotation3d(0,Math.toRadians(67), Math.toRadians(67))));
        ardu = new Camera("Ardu623", new Transform3d(0.67,0.67,0.67, new Rotation3d(0,Math.toRadians(67), Math.toRadians(67))));

    }

    @Override
    public void periodic(){
        List<VisionMeasurement> all = new ArrayList<>(); //Collects all vision measurements from camera
        all.addAll(shutter.estimatePose());
        all.addAll(ardu.estimatePose());


        //How many vision measurements we got this cycle
        Logger.recordOutput("Vision Measurement Count", all.size());

        for(VisionMeasurement m : all){ //Proccesses each vision measurement individually 
            if(m.confidence<0.5){
                Logger.recordOutput("Vision/Rejected" + m.cameraName, true);
            }

            Logger.recordOutput("Vision Accepted" + m.cameraName, true);

            double stdDev = MathUtil.clamp(1.0/m.confidence, 0.05, 2.0); //Converts confidence into uncertainty, higher confidence -> lower noise

            poseEstimator.addVisionMeasurement(m.pose.toPose2d(), m.timeStamp, VecBuilder.fill(stdDev, stdDev, stdDev)); //Feed vision measurement into pose estimator

        }

        Pose2d pose = poseEstimator.getEstimatedPosition(); //Logs final fused robot pose (odometry + vision)
        Logger.recordOutput("Vision Pose", pose);
    }


    //robot pose
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }
    
}
