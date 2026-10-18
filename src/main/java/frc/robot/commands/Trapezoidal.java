//Class with edge case for trapezoidal motion
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;


//Trapezoidal command class for trapezoidal motion, mainly used for linear profiles and linear slides on robot functions.
public class Trapezoidal extends Command{
    //Allows the motion to have a drivetrain object
    private final Drivetrain drivetrain;

    private final double distance;   //Distance required that you want the slide to travel
    private final double max_speed;  //Max speed you want to set it to
    private final double max_accel;  //Max acceleration allowed for the slide

    private double peak_speed;  //Peak speed of the slide at a particular time frame
    private double Ta;          //Time it takes the robot to accelerate
    private double Tmax;        //Time at which the slide is at its max speed
    private double totalTime;   //Total time of the motion

    private double direction;   //Direction of the motion

    private long startTime;     //When the motion starts

    //Constructor initilaizing the drvivetrain, max_speed, max_accel, and distance variables.
    public Trapezoidal(Drivetrain drivetrain, double max_speed, double max_accel, double distance){
        this.drivetrain = drivetrain;
        this.max_speed = Math.abs(max_speed);
        this.max_accel = Math.abs(max_accel);

        direction = Math.signum(distance);  //Figures out if the value is negative or positive, positive direction (+), negative direction(-). Math.signum(determines the sign of the value)
        this.distance = Math.abs(distance); //Absolute distance from the desired position

        calculateProfile(); 

        addRequirements(drivetrain);
    }


    //Trapezoidal math
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
