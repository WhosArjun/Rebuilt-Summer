// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.Trapezoidal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.VisionReal;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionSim;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  
  public RobotState robotState; 
  public final Drivetrain m_drivetrain; 
  public final Shooter m_shooter;
  public final Joystick m_joystick;
  public final VisionIO m_vision;
  public final DriveCommand m_driveCommand; 
  public final Intake m_intake;
  public final Trigger alignTrigger;
  public final Trigger indexTrigger;
  public final Trigger shootTrigger;
  public final Trigger autoShootTrigger;
  public final Trigger feederIn;
  public final Trigger feederOut;
  public final Trigger intakeDownManual;
  public final Trigger intakeUpManual;



  //ButtonBoards
  public final Joystick m_buttonboardA;
  public final Joystick m_buttonboardB;

  public final Trigger trapezoidalTrigger;
  public final Trapezoidal trapezoidalCommand;
  public final Trigger intakeUp;
  public final Trigger intakeDown;
  
               
  private SendableChooser<Command> autoChooser;
  private final Command shootCommand;
  public RobotContainer() {
    m_buttonboardA = new Joystick(0);
    m_buttonboardB = new Joystick(2);
    SmartDashboard.putNumber("Joystick Degree",   2.0);
    m_shooter = new Shooter(58, 36);
    m_drivetrain = new Drivetrain();
    m_joystick = new Joystick(1);
    intakeUpManual = new Trigger(() -> m_buttonboardA.getRawButton(11));
    intakeDownManual = new Trigger(() ->m_buttonboardA.getRawButton(12));
    autoShootTrigger = new Trigger(() ->m_joystick.getRawButton(4));
    shootTrigger = new Trigger(() -> m_buttonboardA.getRawButton(7));
    alignTrigger = new Trigger(() -> m_joystick.getRawButton(6));
    indexTrigger = new Trigger(() -> m_buttonboardA.getRawButton(5));
    m_intake = new Intake(30, 52, ()-> robotState);
    intakeUp = new Trigger(() -> m_buttonboardB.getRawButton(15));
    intakeDown = new Trigger(() -> m_buttonboardB.getRawButton(16));
    feederIn = new Trigger(()-> m_buttonboardA.getRawButton(9));
    feederOut = new Trigger(() -> m_buttonboardA.getRawButton(10));
    robotState = RobotState.NEUTRAL; //instantiate robotState 
    trapezoidalTrigger = new Trigger(() -> m_joystick.getRawButton(6));
    // if(Robot.currentMode){
    //   case REAL -> 
    //   case SIM ->
    // }
    if(!Robot.isReal()){
      m_vision = new VisionSim(m_drivetrain);
    } 
    else {
      m_vision = new VisionReal(m_drivetrain.visionEstimator::addVisionMeasurement);
    }

    
    // m_vision = new VisionIO
    m_driveCommand = new DriveCommand(m_drivetrain, 
                                      () -> {return -m_joystick.getRawAxis(1);},
                                      () -> {return -m_joystick.getRawAxis(0);},
                                      () -> {return -m_joystick.getRawAxis(2);},
                                      () -> {return m_joystick.getRawButton(4);}
                                     ); 

    
    shootCommand = new ParallelCommandGroup (
                Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(m_drivetrain.distanceToRPM()))),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.067),//  TEST TS
                    Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE))
                )
            ).finallyDo((x)->{m_shooter.shooterMotor.set(0); m_shooter.indexMotor.set(0);});
    shootCommand.addRequirements(m_shooter);

    NamedCommands.registerCommand("Shoot", Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(47.6))).finallyDo(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(0))));
    NamedCommands.registerCommand("Shoot2", Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(48.2))).finallyDo(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(0))));

     NamedCommands.registerCommand("Index", Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)).finallyDo(() -> m_shooter.indexMotor.setVoltage(0)));

    NamedCommands.registerCommand("Intake", Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheel.set(Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0));
    }));

    trapezoidalCommand = new Trapezoidal(m_drivetrain,3,3,2);
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand(){
    //String x = autoChooser.getSelected().getName();
    return new PathPlannerAuto("MB");
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_driveCommand);

    alignTrigger.whileTrue(Commands.runOnce(() -> robotState = RobotState.SHOOT));


    trapezoidalTrigger.whileTrue(trapezoidalCommand);

    

    

    intakeUp.whileTrue(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(0));
      m_intake.feederWheel.set(0);
    }));

    intakeUp.or(intakeDown).whileFalse(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(m_intake.intakeMotor.getPosition().getValueAsDouble()));
      m_intake.feederWheel.set(0);
    }, m_intake));

    intakeDown.whileTrue(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheel.set((Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0)));
    }, m_intake));

    feederIn.whileTrue(Commands.run(()-> m_intake.feederWheel.set(Constants.MAX_FLYWHEEL_VOLTAGE)));
    feederIn.onFalse(Commands.runOnce(()->m_intake.feederWheel.set(0)));
    
    feederOut.whileTrue(Commands.run(()-> m_intake.feederWheel.set(-1 * Constants.MAX_FLYWHEEL_VOLTAGE)));
    feederOut.whileFalse(Commands.runOnce(()->m_intake.feederWheel.set(0)));
     
    intakeUpManual.whileTrue(Commands.run(()-> m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(0))));
    intakeDownManual.whileTrue(Commands.run(()->m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION))));


    autoShootTrigger.whileTrue(shootCommand);


    indexTrigger.whileTrue(Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)));
    indexTrigger.whileFalse(Commands.runOnce(() -> m_shooter.indexMotor.setVoltage(0)));

    shootTrigger.whileTrue(Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(57)), m_shooter));
    shootTrigger.whileFalse(Commands.runOnce(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(0))));

  }
  

  public void resetPose(){
    m_drivetrain.swerveDrive.resetOdometry(new Pose2d());
  }


  public enum RobotState{
    NEUTRAL,
    INTAKE,
    OUTTAKE,
    SHOOT;
    
  }
}

