package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Shooter extends SubsystemBase{
    private final TalonFX shooterMotor;
    private final TalonFX indexMotor;
    private double TargetRPS = 0.0;

    private static final double SHOOT_SPEED_RPS = 80.0;
    private static final double INDEX_SPEED = 0.8;
    private static final double SPEED_TOLERANCE = 3.0;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Shooter(int shooterMotorID, int indexMotorID){
        shooterMotor = new TalonFX(shooterMotorID);
        indexMotor = new TalonFX(indexMotorID);
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        indexMotor.setNeutralMode(NeutralModeValue.Brake);
        Slot0Configs shooterPID = new Slot0Configs();
        shooterPID.kP = 0.1;
        shooterPID.kI = 0.0;
        shooterPID.kD = 0.0;
        shooterPID.kV = 0.12;
        shooterMotor.getConfigurator().apply(shooterPID);
    }

    public void spinUp(){
        setShooterSpeed(SHOOT_SPEED_RPS);
    }

    public void setShooterSpeed(double rps){
        TargetRPS = rps;
        shooterMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void stopShooter(){
        TargetRPS = 0;
        shooterMotor.stopMotor();
    }


    public void feed(){
        indexMotor.set(INDEX_SPEED);
    }

    public void reverseFeed(){
        indexMotor.set(-INDEX_SPEED);
    }

    public void stopFeed(){
        indexMotor.stopMotor();
    }

    public double getVelocity(){
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public boolean atSpeed(){
        return Math.abs(getVelocity()-TargetRPS)<SPEED_TOLERANCE;
    }

    public void shoot(){
        spinUp();
        if(atSpeed()){
            feed();
        }
    }

    public void stopAll(){
        stopShooter();
        stopFeed();
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Shooter velocity", getVelocity());
        Logger.recordOutput("Shooter Target RPS", TargetRPS);
        Logger.recordOutput("Shooter atSpeed", atSpeed());
    }
}
