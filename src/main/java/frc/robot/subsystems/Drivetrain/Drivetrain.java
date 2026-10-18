package frc.robot.subsystems.Drivetrain;

import java.io.File;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
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
import frc.robot.subsystems.Intake.Intake;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase{
    public SwerveDrive swerveDrive;
    public final SwerveDrivePoseEstimator visionEstimator; //visionEstimator gets the position, and other variables you get from vision and stores the position through SwerveDrivePoseEstimator
    private final Field2d m_field = new Field2d(); //Creates a simulation of a 2d field
    
    public Drivetrain() {
      //TryCatch statement testing the SwerveParson for properties of the robot, flying through the SwerveModule files
        try{swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Constants.MAX_SPEED,new Pose2d());
    } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException("67 yanis");
    }
    //Initialized visionEstimator by creating a new PoseEstimatorObject, using the variables from the swerveDrive class and files that swerveParson gets.
    visionEstimator = new SwerveDrivePoseEstimator(swerveDrive.kinematics, swerveDrive.getOdometryHeading(), swerveDrive.getModulePositions(), swerveDrive.getPose());    
    configureAuto();
    SmartDashboard.putData("Robot Field Map", m_field);
        
    }
    
    //Configures the autonomous constants, and also gets the autonomous paths to be configured from the Deploy folder from Pathplanner SIMGUI
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
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance, this will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              if(alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }
        

    //pickTarget(), returns a Translation2d, gets the nearest target by using a Vision Pose Estimator
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

//gets the current robot heading from the hub
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

    //hubAngle() using a vision Estimator, getting the position and then finding the error from the bot
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
      SmartDashboard.putNumber("robot relative x", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("robot relative y", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("robot relative theta", speeds.omegaRadiansPerSecond);
      swerveDrive.drive(speeds);
    }

    public String toString(){
        return "Drivetrain";
    }

    public void updateOdom(){
        visionEstimator.update(getGyroRotation(), swerveDrive.getModulePositions());
    }

    
    //Get distance from the hub using a vision pose estimator
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

    //How much do you need the motor to rotate depending on the robot's distance from the hub
    public double distanceToRPM(){
      double[] function = {33.37143, 9.48571};
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
        visionEstimator.update(getGyroRotation(), swerveDrive.getModulePositions());
        SmartDashboard.putNumber("angle error", getHeadingError());
        m_field.setRobotPose(visionEstimator.getEstimatedPosition());
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }
    public void resetEverything(){
        resetPose(new Pose2d());
            zeroGyro();
    }

}
