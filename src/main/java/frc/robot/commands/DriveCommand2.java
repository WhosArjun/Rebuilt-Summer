package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand2 extends Command{
    DoubleSupplier xTranlationSupplier;
    DoubleSupplier yTranslationSupplier;
    DoubleSupplier thetaTranslationSupplier;
    Drivetrain drivetrain;

    public DriveCommand2(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranlationSupplier, DoubleSupplier thetaTranslationSupplier, Drivetrain drivetrain){
        this.xTranlationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranlationSupplier;
        this.thetaTranslationSupplier = thetaTranslationSupplier;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double joystickX = xTranlationSupplier.getAsDouble();
        double joystickY = yTranslationSupplier.getAsDouble();
        double joystickTheta = thetaTranslationSupplier.getAsDouble();

        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(
            deadzone(xTranlationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisVelocity()),
            deadzone(yTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisVelocity()),
            deadzone(thetaTranslationSupplier.getAsDouble(),0.05) * Math.abs(drivetrain.swerveDrive.getMaximumChassisAngularVelocity())
         )); 

    }

    public double deadzone(double number, double deadband){
        if(Math.abs(number)<deadband){
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



}
