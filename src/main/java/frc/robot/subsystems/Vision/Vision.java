package frc.robot.subsystems.Vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public Camera shutter;  
    public Camera ardu;
    public boolean visionOn = true;
    BiConsumer<Pose2d, Double> updateDrivetrain;
    public Vision(BiConsumer<Pose2d, Double> updateDrivetrain){
        shutter = new Camera("Shutter623", new Transform3d(), new Rotation3d());
        ardu = new Camera("Arducam623", new Transform3d(), new Rotation3d());
        this.updateDrivetrain = updateDrivetrain;
    }
}
