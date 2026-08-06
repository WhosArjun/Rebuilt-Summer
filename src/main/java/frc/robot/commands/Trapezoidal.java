//Class with edge case for trapezoidal motion
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Trapezoidal extends Command{
    private final Drivetrain drivetrain;

    private final double distance;
    private final double max_speed;
    private final double max_accel;

    private double peak_speed;
    private double Ta;
    private double Tmax;
    private double totalTime;

    private double direction;

    private long startTime;

    public Trapezoidal(Drivetrain drivetrain, double max_speed, double max_accel, double distance){
        this.drivetrain = drivetrain;
        this.max_speed = Math.abs(max_speed);
        this.max_accel = Math.abs(max_accel);

        direction = Math.signum(distance);
        this.distance = Math.abs(distance);

        calculateProfile();

        addRequirements(drivetrain);
    }

    private void calculateProfile(){
        double accelTimeToMax = max_speed/max_accel;
        double accelDistance = 0.5*max_accel*accelTimeToMax*accelTimeToMax;

        double minDistanceForTrapezoid = accelDistance*2;

        //Edge Case (If not enough distance to reach max speed)
        if(distance<minDistanceForTrapezoid){
            peak_speed = Math.sqrt(distance*max_accel);
            Ta = peak_speed/max_accel;
            Tmax = 0;
        }
        else{
            peak_speed = max_speed;
            Ta = accelTimeToMax;
            double cruiseDistance = distance - minDistanceForTrapezoid;
            Tmax = cruiseDistance / peak_speed;
        }

        totalTime = (2*Ta) + Tmax;
    }

    private double getTime(){
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    private double speed(){
        double t = getTime();
        //Acceleration
        if(t<Ta){
            return max_accel * t;
        }
        //Constant Velocity
        else if (t<Ta+Tmax){
            return peak_speed;
        }
        //Deceleration
        else if(t<totalTime){
            double decelTime = t - (Ta+Tmax);
            return peak_speed - (max_accel * decelTime);
        }
        return 0;
    }

    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        double currentSpeed = speed() * direction;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(currentSpeed, 0, 0);
        drivetrain.swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    @Override
    public boolean isFinished(){
        return getTime() >= totalTime;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.swerveDrive.drive(new ChassisSpeeds(0*67,0,0));
    }
    
}
