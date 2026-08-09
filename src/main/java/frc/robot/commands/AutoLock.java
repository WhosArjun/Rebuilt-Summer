package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoLockConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class AutoLock extends Command{
    private final Drivetrain drivetrain;
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;

    private final PIDController thetaController;

    public AutoLock(Drivetrain drivetrain, DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier){
        this.drivetrain = drivetrain;
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;

        thetaController = new PIDController(AutoLockConstants.kP,
                                            AutoLockConstants.kI,
                                            AutoLockConstants.kD);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        thetaController.setTolerance(Math.toRadians(AutoLockConstants.ANGLE_TOLERANCE_DEGREES));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        thetaController.reset();
    }


    @Override
    public void execute(){
        Pose2d robotPose = drivetrain.getPose();
        Translation2d target;

        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            target = Constants.redHub; 
        } else {
            target = Constants.blueHub; 
        }

        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d difference = target.minus(robotPosition);

        double targetAngle = Math.atan2(difference.getY(), difference.getX());

        double currentAngle = robotPose.getRotation().getRadians();

        double omega = thetaController.calculate(currentAngle, targetAngle);

        omega = Math.max(-drivetrain.swerveDrive.getMaximumChassisAngularVelocity(), Math.min(omega,
        drivetrain.swerveDrive.getMaximumChassisAngularVelocity()));

        double xSpeed = DriveCommand.deadzone(xTranslationSupplier.getAsDouble(), 0.05) * drivetrain.swerveDrive.getMaximumChassisVelocity();

        double ySpeed = DriveCommand.deadzone(yTranslationSupplier.getAsDouble(), 0.05) * drivetrain.swerveDrive.getMaximumChassisVelocity();


        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(xSpeed, ySpeed, omega));

        SmartDashboard.putNumber("AutoLock(Target Angle)", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Autolock/Robot Angle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("Autolock, Angle Error", Math.toDegrees(thetaController.getPositionError()));
        SmartDashboard.putNumber("Autolock, omega", omega);
        SmartDashboard.putBoolean("Autolock, at target", thetaController.atSetpoint());

    }

    @Override
    public void end(boolean interrupted){
        drivetrain.swerveDrive.driveFieldOriented(new ChassisSpeeds(0,0,0));
        thetaController.reset();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override 
    public boolean runsWhenDisabled(){
        return false;
    }

    @Override 
    public Set<Subsystem> getRequirements(){
        HashSet<Subsystem> requirements = new HashSet<>();
        requirements.add(drivetrain);
        return requirements;
    }



}
