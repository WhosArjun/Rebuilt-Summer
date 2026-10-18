package frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RobotState;

public class Shooter extends SubsystemBase{
    public TalonFX shooterMotor; 
    public TalonFX indexMotor;
    public TalonFX shooter2Motor;

    //Constructor with the ID's of all the motors
    public Shooter(int shooterMotorID, int indexMotorID, int shooter2MotorID){
        //Applying ID's
        this.shooterMotor = new TalonFX(shooterMotorID);
        this.indexMotor = new TalonFX(indexMotorID);
        this.shooter2Motor = new TalonFX(shooter2MotorID);

        //Applies the state of Brake or Coast to all the motors (Look at intake file for BRAKE and COAST do)
        this.shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        this.indexMotor.setNeutralMode(NeutralModeValue.Brake);
        this.shooter2Motor.setNeutralMode(NeutralModeValue.Coast);
    }
}
