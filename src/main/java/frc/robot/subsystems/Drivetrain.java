package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase{
    public SwerveDrive swerveDrive;
    
    public Drivetrain() {
        try{swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Constants.MAX_SPEED,new Pose2d());
    } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException("67 yanis");
    }
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

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose){
        swerveDrive.resetOdometry(pose);
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
}
