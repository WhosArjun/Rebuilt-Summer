package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveDrive;

public class Trapezoidal extends Command{
    private Drivetrain drivetrain;
    private long startTime;
    private double MAX_SPEED;
    private double MAX_ACCEL;
    private double distance;
    private double goalX;
    private double Ta;
    private double twoTriangleArea;
    private double Tmax;
    private double areaOfRectangle;
    private double t1;
    
    public Trapezoidal(Drivetrain drivetrain,double MAX_SPEED, double MAX_ACCEL, double distance, double goalX
                    , double Ta,double Tmax, double t1){
        this.MAX_SPEED = MAX_SPEED;
        this.MAX_ACCEL = MAX_ACCEL;
        this.distance = distance;
        this.goalX = goalX;
        this.Ta = (MAX_SPEED)/Math.tan(MAX_ACCEL);
        twoTriangleArea = (MAX_SPEED)*Ta;
        this.Tmax = (goalX)/MAX_SPEED-Ta;
        areaOfRectangle = Tmax * MAX_SPEED;
        this.t1 = Ta+Tmax;
        this.drivetrain = drivetrain;
    }
    //MAX_SPEED
    public double getMaxSpeed(){
        return MAX_SPEED;
    }
    //MAX_ACCELERATION
    public double getMaxAccel(){
        return MAX_ACCEL;
    }
    //Base of the triangle
    public double getTa(){
        return Ta;
    }
    //Top width of the trapezoid
    public double getTmax(){
        return Tmax;
    }
    //Peak of the MAX_SPEED//ACCELERATION on the right side
    public double getT1(){
        return t1;
    }
    public double getTime(){
        long current = System.currentTimeMillis();
        return (current-startTime)/1000.0;
    }

    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
    }
    
    double accelerationFunction;
    double pieceWiseFunctionThree;
    double decelerationFunction;
    public double speed(){
        if(getTime()<getTa()){//when time is less than ta
            accelerationFunction = getMaxAccel()*getTime();
            return accelerationFunction;
        }
        else if(getTime()>getTa()&&getTime()<getTa()+getTmax()){//when time is greater than ta and less than ta+tmax
            pieceWiseFunctionThree = getMaxSpeed();
            return pieceWiseFunctionThree;
        }
        else if(getTime()>getTa()+getTmax()){//also wrong baka
            decelerationFunction = -(getMaxAccel())*(getTime()-getT1())+getMaxSpeed();//when time is greater than ta+tmax
            return decelerationFunction;
        }
        return 0;
    }
    public void execute(){
        ChassisSpeeds chassisspeed = new ChassisSpeeds(speed(), 0,0);
        drivetrain.swerveDrive.drive(chassisspeed);
    }
        
    //edge case (when there is no trapezoid), when goalX<distance needed to accelerate and decelerate
    
    

    
    
    
    
    
    
    
    /*public double twoTriangleArea(){
        double Ta = (MAX_SPEED)/Math.tan(MAX_ACCEL); //Hypotenuse of triangle
        double base = (MAX_SPEED)*Ta;
        return base;
    }

    public double rectangleArea(){
        double Ta = (MAX_SPEED)/Math.tan(MAX_ACCEL);
        double Tmax = (goalX)/MAX_SPEED-Ta;
        return Tmax;
    }
        */

    
}
