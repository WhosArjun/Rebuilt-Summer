package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

        double currentAngle = robotPose.getRotation().getRadians();
    }

}
