package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.RobotState;

public class Intake extends SubsystemBase{
    public TalonFX intakeMotor;  //Motor that controls the intake
    public TalonFX feederWheelR; //Feeder wheel on the right of the intake
    public TalonFX feederWheelL; //Feeder wheel on the left of the intake
    public Supplier<RobotState> robotStateSupplier;

    //Constructor containing the ID of the motors, and state of the robot in which the intake acts accordingly
    public Intake(int intakeWheelId, int feederWheelIDR, int feederWheelIDL, Supplier<RobotState> robotStateSupplier){
        this.intakeMotor = new TalonFX(intakeWheelId);
        this.feederWheelR = new TalonFX(feederWheelIDR);
        this.feederWheelL = new TalonFX(feederWheelIDL);
        this.robotStateSupplier = robotStateSupplier;
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake); //Allows the intake motor to haev a brake, and not control the intake whenever it wants
        this.feederWheelR.setNeutralMode(NeutralModeValue.Coast);
        this.feederWheelL.setNeutralMode(NeutralModeValue.Coast); //Coast mode, allows the intake to apply motor speed whenevr
        intakeMotor.setPosition(0);

        //TalonFX motor configuration, and MotionMagic configuration of the robot
        TalonFXConfiguration configs = new TalonFXConfiguration();
        var mm = configs.MotionMagic;
        mm.MotionMagicCruiseVelocity = 1;  
        mm.MotionMagicAcceleration = 0.5;  
        mm.MotionMagicJerk = 0;
        var slot0 = configs.Slot0;
        slot0.kP = 1.5; //Agressiveness, multiples the current error by a gain to determine how much power to send to the motor
        slot0.kV = 0.0; //Velocity, speed and direction of the motor
        slot0.kS = 0.0; //Speed at which the motor rotates

        intakeMotor.getConfigurator().apply(configs); //Apply configuration setup to the intake motor
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("IntakePosition", intakeMotor.getPosition().getValueAsDouble());
    }
}
