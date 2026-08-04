package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public Camera shutter;
    public Camera ardu;
    public boolean visionOn = true;
    BiConsumer<Pose2d, Double> updateDrivetrain;

    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(0.263456, 0.2980182, 0.512445, new Rotation3d(0, Math.toRadians(-18.3), Math.toRadians(11))));
        ardu = new Camera("Arducam623", new Transform3d(0.263092, -0.300228, 0.5041392, new Rotation3d(0, Math.toRadians(-9.2), Math.toRadians(-10))));
        this.updateDrivetrain = updateDrivetrain;
    }

    public void periodic(){
        SmartDashboard.putBoolean("vision", visionOn);

        if(!visionOn) return;
        List<VisionReading> readings = new ArrayList<>();

        for(VisionReading reading : shutter.estimatePose()){
            readings.add(reading);
        }
        for(VisionReading reading : ardu.estimatePose()){
            readings.add(reading);
        }
        for(VisionReading reading : readings){
            updateDrivetrain.accept(reading.getPose2d(), reading.getTimeStampSeconds());
        }
    }
}
