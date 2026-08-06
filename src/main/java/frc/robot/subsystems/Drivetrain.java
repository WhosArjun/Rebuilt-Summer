package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase{
    public SwerveDrive swerveDrive;
    public final SwerveDrivePoseEstimator visionEstimator;
    
    public Drivetrain() {
        try{swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Constants.MAX_SPEED,new Pose2d());
    } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException("67 yanis");
    }
    visionEstimator = new SwerveDrivePoseEstimator(swerveDrive.kinematics, swerveDrive.getOdometryHeading(), swerveDrive.getModulePositions(), swerveDrive.getPose());    
    configureAuto();
        
    }
    
    private void configureAuto(){
        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        }
        catch(Exception e){
            throw new RuntimeException("Failed to grab Pathplanner");
        }


        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> false,
            this

        );
    }

    public Rotation2d getGyroRotation(){
        return swerveDrive.getYaw();
    }
    
    public void addVisionMeasurement(Pose2d pose, double timestamp, double stdDev){
        swerveDrive.addVisionMeasurement(pose, timestamp, VecBuilder.fill(stdDev,stdDev,stdDev));
    }

    public Pose2d getPose(){
        return visionEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose){
        swerveDrive.resetOdometry(pose);

        visionEstimator.resetPosition(getGyroRotation(), swerveDrive.getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        swerveDrive.drive(speeds);
    }

    public String toString(){
        return "Drivetrain";
    }

    public void updateOdom(){
        visionEstimator.update(getGyroRotation(), swerveDrive.getModulePositions());
    }
}
