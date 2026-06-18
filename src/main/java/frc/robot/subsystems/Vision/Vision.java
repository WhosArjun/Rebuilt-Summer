package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Vision extends SubsystemBase{
    private final Drivetrain drivetrain;
    private final Camera shutter;
    private final Camera ardu;
    private int currentTagId;
    
    public Vision(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        Transform3d shutterCamPose = new Transform3d(0.2,0.2,0.3, new Rotation3d(0,Math.toRadians(25),0));
        Transform3d arduCamPose = new Transform3d(0.2,0.2,0.3, new Rotation3d(0,Math.toRadians(25),0));
        shutter = new Camera("Shutter623", shutterCamPose);
        ardu = new Camera("Ardu623", arduCamPose);
        currentTagId = -1;
    }

    @Override
    public void periodic(){
        List<VisionMeasurement> all = new ArrayList<>();
        all.addAll(shutter.estimatPose());
        all.addAll(ardu.estimatPose());

        Logger.recordOutput("Vision/Measurement Count", all.size());

        for(VisionMeasurement m : all){
            currentTagId = m.getTagId();
            if(m.confidence<0.5){
                Logger.recordOutput("Vision" + m.cameraName + "Rejected", true);
                continue;
            }

            Logger.recordOutput("Vision" + m.cameraName + "Accepted", true);

            double stdDev = MathUtil.clamp(1.0/m.confidence, 0.05, 2.0);

            drivetrain.addVisionMeasurement(m.pose.toPose2d(), m.timeStamp, stdDev);

        }
        Logger.recordOutput("Vision Position", drivetrain.getPose());
    }
    
    public Optional<Pose2d> getCurrentTagPose(){
        if(currentTagId == -1){
            return Optional.empty();
        }
        return Constants.kTagLayout.getTagPose(currentTagId).map(p->p.toPose2d());
    }


    public int getCurrentTagId(){
        return currentTagId;
    }
    public Pose2d getPose(){
        return drivetrain.getPose();
    }
}
