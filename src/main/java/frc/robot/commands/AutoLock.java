package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.Vision;

public class AutoLock extends Command{
    private final Drivetrain drivetrain;
    private final Vision vision;
    private Pose2d targetPose;
    
    private final PIDController xPID = new PIDController(2.5,0,0);
    private final PIDController yPID = new PIDController(2.5,0,0);
    private final PIDController thetaPID = new PIDController(4.0,0,0);

    public AutoLock(Drivetrain drivetrain, Vision vision){
        this.drivetrain = drivetrain;
        this.vision = vision;
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        Optional<Pose2d> tagPose = vision.getCurrentTagPose();
        if(tagPose.isEmpty()){
            targetPose = null;
            return;
        }
        //Robot will try to stop 0.75 meters in front of the tag
        targetPose = tagPose.get().transformBy(new Transform2d(-0.75, 0.0, Rotation2d.kZero)); 
    }

    @Override
    public void execute(){
        if(targetPose==null){
            return;
        }
        Pose2d current = drivetrain.getPose();
        double vx = xPID.calculate(current.getX(), targetPose.getX());
        double vy = yPID.calculate(current.getY(), targetPose.getY());
        double omega = thetaPID.calculate(current.getRotation().getRadians(), targetPose.getRotation().getRadians());

        drivetrain.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(vx,vy,omega,current.getRotation()));

    }
    //5 centimeter tolerance
    @Override
    public boolean isFinished(){
        if(targetPose == null){
            return true;
        }
        Pose2d current = drivetrain.getPose();
        return current.getTranslation().getDistance(targetPose.getTranslation())<0.05;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.driveRobotRelative(new ChassisSpeeds());
    }
}
