package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase{
    public SwerveDrive swerveDrive;
    public final SwerveDrivePoseEstimator visionEstimator;
    private final Field2d m_field = new Field2d();
    
    public Drivetrain() {
        try{swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Constants.MAX_SPEED,new Pose2d());
    } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException("67 yanis");
    }
    visionEstimator = new SwerveDrivePoseEstimator(swerveDrive.kinematics, swerveDrive.getOdometryHeading(), swerveDrive.getModulePositions(), swerveDrive.getPose());    
    configureAuto();
    SmartDashboard.putData("Robot Field Map", m_field);
        
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

    public Translation2d pickTarget(){
      Pose2d pose = visionEstimator.getEstimatedPosition();
      if(DriverStation.getAlliance().get()==Alliance.Red){
        if(pose.getX()>11.99){ // red alliance and in alliance zone
        SmartDashboard.putString("target", "redHub");
          return Constants.redHub;
        }
        else if (pose.getY()>4.03){ // red alliance and not in alliance zone
          SmartDashboard.putString("target", "redPassUp");
          return Constants.redPassUp;
        }
        else {
          SmartDashboard.putString("target", "redPassDown");
          return Constants.redPassDown;
        }
      }
      else {
        if(pose.getX()<4.55){// blue alliance and in alliance zone
          SmartDashboard.putString("target", "blueHub");
          return Constants.blueHub; 
        }
        else if (pose.getY()>4.03) {// blue alliance and not in alliance zone
          SmartDashboard.putString("target", "bluePassUp");
          return Constants.bluePassUp;
        }
        else {
          SmartDashboard.putString("target", "bluePassDown");
          return Constants.bluePassDown;
        }
      }
    }


    /*  public double getHeadingError() {
        Translation2d target = pickTarget();
        
        Pose2d currentPose = visionEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("currentPose x", currentPose.getX());
        SmartDashboard.putNumber("currentPose y", currentPose.getY());

        double dx = target.getX() - currentPose.getX();
        SmartDashboard.putNumber("dx", dx);
        double dy = target.getY() - currentPose.getY();
        SmartDashboard.putNumber("dy", dy);

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
        SmartDashboard.putNumber("targetAngle", targetAngle);

        double error = targetAngle - currentPose.getRotation().getDegrees();;
        error = (error+360)%360; 
        SmartDashboard.putNumber("error", error);
        // error = (error + 180) % 360;
        if (error > 180) error -= 360;

        // error -= 180;
        // System.out.println(error);
        return error;
    }
        */
public double getHeadingError() {
    Pose2d robot = visionEstimator.getEstimatedPosition();
    Translation2d target = pickTarget();

    //dy and dx are the differences in the x and y coordinates between the robot and the target
    double dx = target.getX() - robot.getX();
    double dy = target.getY() - robot.getY();
    //Find the angle to the target using atan2, which returns the angle in radians between the positive x axis and the point (dx,dy), we then convert it to degrees
    double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
    double robotAngle = robot.getRotation().getDegrees();
    //Comparing the error to where we are actually pointing 
    double error = MathUtil.inputModulus(targetAngle-robotAngle, -180,180);

    SmartDashboard.putNumber("Autolock target angle", targetAngle);
    SmartDashboard.putNumber("Autolock robot angle", robotAngle);
    SmartDashboard.putNumber("Autolock error", error);
    return error;
}
    public double hubAngle() {
      Pose2d currentPose = visionEstimator.getEstimatedPosition();
      double x = currentPose.getX();
      double y = currentPose.getY();
      //Translation2d target
      double angleToHub = Math.toDegrees(Math.atan((y-Constants.redHub.getY())/(x-Constants.redHub.getX())) % 360);
      return angleToHub;
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

    public double distance(){
    Pose2d currentPose = visionEstimator.getEstimatedPosition();
    double x = currentPose.getX();
    SmartDashboard.putNumber("botx", x);
    double y = currentPose.getY();
    SmartDashboard.putNumber("boty", y);
    if(DriverStation.getAlliance().get()==Alliance.Blue){
      SmartDashboard.putNumber("distance", Math.sqrt(Math.pow((x-Constants.blueHub.getX()),2) + Math.pow((y-Constants.blueHub.getY()),2)));
        return Math.sqrt(Math.pow((x-Constants.blueHub.getX()),2) + Math.pow((y-Constants.blueHub.getY()),2));
    }
        else{
          SmartDashboard.putNumber("distance", Math.sqrt((x-Constants.redHub.getX())*(x-Constants.redHub.getX())+(y-Constants.redHub.getY())*(y-Constants.redHub.getY())));
      return Math.sqrt((x-Constants.redHub.getX())*(x-Constants.redHub.getX())+(y-Constants.redHub.getY())*(y-Constants.redHub.getY()));
        }
    }


    public double distanceToRPM(){
      double[] function = {33.37143, 9.48571};//0th coeff, 1st, 2nd, etc
      // double distance = distance();
      double distance = distance();
      double sum = 0;
      for(int i = 0;i < function.length; i++){
        sum += function[i]*Math.pow(distance,i);
      }
      SmartDashboard.putNumber("regression Output", sum);
      return sum;
    }

    @Override
    public void periodic(){
        /*SmartDashboard.putNumber("Angle error", getHeadingError());
        visionEstimator.update(getGyroRotation(), swerveDrive.getModulePositions());
        m_field.setRobotPose(visionEstimator.getEstimatedPosition());
        */
        visionEstimator.update(getGyroRotation(), swerveDrive.getModulePositions());
        SmartDashboard.putNumber("angle error", getHeadingError());
        m_field.setRobotPose(visionEstimator.getEstimatedPosition());

    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }
    public void resetEverything(){
        resetPose(new Pose2d());
            resetEverything();
    }

}
