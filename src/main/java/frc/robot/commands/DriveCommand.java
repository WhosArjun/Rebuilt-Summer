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
import frc.robot.subsystems.Drivetrain;
import swervelib.math.SwerveMath;

public class DriveCommand extends Command{
    private Drivetrain drivetrain;
    private DoubleSupplier xTranslationSupplier;
    private DoubleSupplier yTranslationSupplier;
    private DoubleSupplier thetaTranslationSupplier;
   
    //private Supplier<RobotState> stateSupplier;
    private PIDController thetaController;
    //Constructor contains important information such as robotstate, translation and PID values.
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier,
                        DoubleSupplier yTranslationSupplier, DoubleSupplier thetaTranslationSupplier){
                            this.drivetrain = drivetrain;
                            this.xTranslationSupplier = xTranslationSupplier;
                            this.yTranslationSupplier = yTranslationSupplier;
                            this.thetaTranslationSupplier = thetaTranslationSupplier;
                        
                           // this.thetaController = new PIDController(kP, kI, kD);
                           // this.stateSupplier = stateSupplier;
                            //this.thetaController = new PIDController(kP, kI, kD);

    }
    //Supplier<RobotState> stateSupplier, double kP, double kI, double kD

    @Override
    public void initialize(){

    }

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
            deadzone(thetaTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisAngularVelocity())
         )); 


    }
    
    public static double deadzone(double number, double deadband){ 
        if (Math.abs(number)<deadband){
            return 0;
        }
        return number;
    }
    

    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return false;
    }

    public boolean runsWhenDisabled(){
        return false;
    }
    @Override
    public Set<Subsystem> getRequirements(){
        HashSet<Subsystem> requirement = new HashSet<>();
        requirement.add(drivetrain);
        return requirement;
    }

    
    /*public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
        return run(() -> {
        Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()),0.8);
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        })
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
        return run(() -> {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                        translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(), true, false);

        });
    }
        */
}
