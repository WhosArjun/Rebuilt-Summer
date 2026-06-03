package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TrapezoidalEdge extends Command{
    private Drivetrain drivetrain;
    private double distance;
    private double maxAccel;
    private double maxSpeed;

    private double peakSpeed; //sometimes peakspeed<maxspeed and peakspeed=maxspeed
    private double Ta; //time spent accelerating
    private double Tmax; //time spent cruising at a constant speed rectangle part, int triangle profile Tmax = 0
    private double totalTime; //entire motion duration
    //accelerate+cruise+decelerate : 2Ta+Tmax
    private double direction; //Allows forward and backword (distance could be -6 or 7)
    private long startTime;

    public TrapezoidalEdge(Drivetrain drivetrain, double maxSpeed,
                        double maxAccel, double distance){
                            this.drivetrain = drivetrain;
                            this.maxSpeed = Math.abs(maxSpeed); //Pos
                            this.maxAccel = Math.abs(maxAccel); //Pos
                            direction = Math.signum(distance); //+1 if pos, -1 if neg, 0 if zero
                            this.distance = Math.abs(distance); //distance pos
                            calculateProfile(); //computes all timing values 
                            addRequirements(drivetrain); //prevents multiple commands from controlling drivetrain simultaneously
     }
    public void calculateProfile(){
        double accelTimeToMax = maxSpeed/maxAccel; // t = v/a (4ms / 2ms^2) = 2s
        double accelDistance = 0.5*maxAccel*Math.pow(accelTimeToMax,2); //distance traveled during acceleration (d=1/2*a*t^2)
        double minDistanceForTrapezoid = accelDistance*2; //min distance for trapezoid
        //EDGE CASE CHECK
        //not enough distance to reach maxspeed
        if(distance<minDistanceForTrapezoid){ //can robot reach maxspeed
            //triangle profile
            peakSpeed = Math.sqrt(distance*maxAccel); //vpeak = sqrt(distance*a) - google ai
            Ta = peakSpeed/maxAccel; //peakSpeed/maxAccel 
            Tmax = 0; //no rectangle exists
        }
        else{ //if distance is large enough for maxspeed to be reached
            //noraml trapezoid profile
            peakSpeed = maxSpeed;
            Ta = accelTimeToMax;
            double cruiseDistance = distance-minDistanceForTrapezoid; //subtracting triangle parts
            Tmax = cruiseDistance/peakSpeed; //t = d/v (cruise time)

        }
        totalTime = (2*Ta)+Tmax; //entire motion duration
    } 
    private double getTime(){
        double time = (System.currentTimeMillis()-startTime)/1000.0; //return time since command started
        return time;
    }
    private double speed(){
        double t = getTime();
        if(t<Ta){ //robot speeding up
            return maxAccel*t; //v=at (velocity formula)
        }
        else if(t>Ta && t<Ta+Tmax){ //cruise face
            return peakSpeed; //flat trapezoid part
        }
        else if(t>Ta+Tmax){ //robot decelerating
            double deceltime = t-(Ta+Tmax); //time since decel begins
            return peakSpeed-(maxAccel*deceltime); //decreasing velocity
        }
        return 0; //robot stop
    }
    @Override
    public void initialize(){
        startTime = System.currentTimeMillis(); //start command timer
    }
    @Override
    public void execute(){
        double currentSpeed = speed()*direction; //without this robot only goes forward
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(currentSpeed, 0,0); //make chassis speeds
        drivetrain.swerveDrive.driveFieldOriented(chassisSpeeds);
    }
    @Override
    public boolean isFinished(){
        return getTime()>=totalTime; //ends command after motion completes
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.swerveDrive.drive(new ChassisSpeeds(0*67,0,0)); //stops robot when command ends
    }
    
}
