package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import swervelib.math.SwerveMath;

public class DriveCommand extends Command{
    private Supplier<Boolean> autoShooter; //autoChooser allows you to choose what autonomous you would like to run in elastic
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier; //constant supplier of the y position of the robot
    private DoubleSupplier yTranslationSupplier; //constant supplier of the x position of the robot
    private DoubleSupplier thetaTranslationSupplier;  //constant supplier of the theta (heading) of the robot
    private PIDController pidController; //PID values tuned controller


    //Constructuer initializing all the instance variable of DriveCommand
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier thetaTranslationSupplier, Supplier<Boolean> autoShooter){
                            this.drivetrain = drivetrain;
                            this.xTranslationSupplier = xTranslationSupplier;
                            this.yTranslationSupplier = yTranslationSupplier;
                            this.thetaTranslationSupplier = thetaTranslationSupplier;
                            this.autoShooter = autoShooter;
                            pidController = new PIDController(0.075, 0, 0); //Initializing PID values with kP, kI, kD, proportional, integral, derivative

    }
    @Override
    public void initialize(){
        //Initilaizing the command/start of the skeleton code
    }


    //Constantly supplies the joystick (x,y,z) values, and puts them on smartDashboard. Allows the robot to drive fieldOriented by getting the chassisVelocity values and applying a deadzone to the joystick.
    @Override
    public void execute(){ 
        double joystickX = xTranslationSupplier.getAsDouble();
        double joystickY = yTranslationSupplier.getAsDouble();
        double joystickTheta = thetaTranslationSupplier.getAsDouble();
        SmartDashboard.putNumber("joystick y : ", joystickY);
        SmartDashboard.putNumber("joystick x : ", joystickX);
        SmartDashboard.putNumber("joystick theta : ", joystickTheta);

        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(
            deadzone(xTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisVelocity()),
            deadzone(yTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisVelocity()),
           getThetaVelocity()
         )); 


    }

    //If the autolock button on the Joystick is on, you run autolock, and you apply the autoshooter button. Calculates the heading of the bot to the hub, and applies the error needed to adjust.
    public double getThetaVelocity(){
        SmartDashboard.putBoolean("buttonOn", autoShooter.get());
        if(autoShooter.get() == true){
            double thetaError = drivetrain.getHeadingError();
            SmartDashboard.putNumber("theta error", thetaError);
            double thetaOutput = pidController.calculate(thetaError, 0);
            return -thetaOutput; //test this
        }
        else{
            return deadzone(thetaTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisAngularVelocity());
        }
    }
    
    //Deadzone of the joystick for people with butter fingers and minor errors in the joystick.
    public static double deadzone(double number, double deadband){ 
        if (Math.abs(number)<deadband){
            return 0;
        }
        return number;
    }
    

    public void end(boolean interrupted){
        //End of the command base
    }

    //If the command is finished running
    public boolean isFinished(){
        return false;
    }

    //Doesn't allow the robot to run when disabled
    public boolean runsWhenDisabled(){
        return false;
    }

    //Gets a hashset for the robot command, and continuously adds DriveCOmmand to be called.
    @Override
    public Set<Subsystem> getRequirements(){
        HashSet<Subsystem> requirement = new HashSet<>();
        requirement.add(drivetrain);
        return requirement;
    }
}