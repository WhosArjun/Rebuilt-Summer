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
    public TalonFX intakeMotor;
    public TalonFX feederWheel;
    public Supplier<RobotState> robotStateSupplier;

    public Intake(int intakeWheelId, int feederWheelID, Supplier<RobotState> robotStateSupplier){
        this.intakeMotor = new TalonFX(intakeWheelId);
        this.feederWheel = new TalonFX(feederWheelID);
        this.robotStateSupplier = robotStateSupplier;
        this.intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        this.feederWheel.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.setPosition(1,1.0);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        var mm = configs.MotionMagic;
        mm.MotionMagicCruiseVelocity = 13;
        mm.MotionMagicAcceleration = 7;
        mm.MotionMagicJerk = 0;
        var slot0 = configs.Slot0;
        slot0.kP = 3;
        slot0.kV = 0.0;
        slot0.kS = 0.0;

        intakeMotor.getConfigurator().apply(configs);
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Position",intakeMotor.getPosition().getValueAsDouble());

        /* 
        RobotState state = robotStateSupplier.get();

        if(state == RobotState.INTAKE){
            intake();
        }
        else if (state == RobotState.OUTTAKE){
            outtake();
        }
        else{
            stop();
        }
        */
    }
}
