package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private final Camera shutter;
    private final Camera ardu;
    private boolean visionEnabled = true;

    private final Consumer<VisionReadings> visionConsumer;
    public Vision(Consumer<VisionReadings> visionConsumer){
        this.visionConsumer = visionConsumer;
        shutter = new Camera("Shutter623", new Transform3d(0.67,0.67,0.67, new Rotation3d(0, Math.toRadians(0.67), Math.toRadians(0.67))));
        ardu = new Camera("Ardu623", new Transform3d(0.67,0.67,0.67, new Rotation3d(0, Math.toRadians(0.67), Math.toRadians(0.67))));
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Vision Enabled", visionEnabled);
        if(!visionEnabled){
            Logger.recordOutput("Shutter connected", shutter.isCameraConnected());
            Logger.recordOutput("Ardu Connected", ardu.isCameraConnected());

            List<VisionReadings> allReadings = collectAllReadings();
            Logger.recordOutput("Vision Reading Count", allReadings.size());

            VisionReadings best = selectBestReading(allReadings);

            if(best!=null){
                Logger.recordOutput("Vision Best Position", best.getPose2d());
                Logger.recordOutput("Vision best timeStamp", best.getTimestampSeconds());
                Logger.recordOutput("Vision best camera", best.getName());
                Logger.recordOutput("Vision best tagCount", best.getTagCount());
                Logger.recordOutput("Vision Best ambiguity", best.getAmbiguity());

                visionConsumer.accept(best); 
            }
        }

    }

    private List<VisionReadings> collectAllReadings(){
            List<VisionReadings> readings = new ArrayList<>();
            readings.addAll(shutter.estimatePose());
            readings.addAll(ardu.estimatePose());
            return readings;
    }

    private VisionReadings selectBestReading(List<VisionReadings> readings){
        VisionReadings best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for(VisionReadings r : readings){
            double score = r.getTagCount()/(1+r.getAmbiguity());
            if(score>bestScore){
                bestScore = score;
                best = r;
            }
        }
        return best;
    }

    public void setVisionEnabled(boolean enabled){
        this.visionEnabled = enabled;
    }

    public boolean isVisionEnabled(){
        return visionEnabled;
    }
    
}
