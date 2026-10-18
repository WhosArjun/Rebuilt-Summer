// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.Trapezoidal;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.VisionReal;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionSim;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  //Variables for all the buttons we use
  public RobotState robotState; 
  public final Drivetrain m_drivetrain; 
  public final Shooter m_shooter;
  public final Joystick m_joystick;
  public final VisionIO m_vision;
  public final DriveCommand m_driveCommand; 
  public final Intake m_intake;
  public final Trigger indexTrigger;
  public final Trigger shootTrigger;
  public final Trigger autoShootTrigger;
  public final Trigger indexIn;
  public final Trigger flywheelIn;
  public final Trigger feederIn;
  public final Trigger feederOut;
  public final Trigger intakeDownManual;
  public final Trigger intakeUpManual;
  public final Trigger intakeUp;
  public final Trigger intakeDown;
  public final Trigger navxResetButton;
  public final Trigger coastMode;
  public final Trigger zeroIntake;
  

  //ButtonBoards
  public final Joystick m_buttonboardA;
  public final Joystick m_buttonboardB;

  //public final Trigger trapezoidalTrigger;
  //public final Trapezoidal trapezoidalCommand;



  
               
  //private SendableChooser<Command> autoChooser;
  private final Command shootCommand;
  public RobotContainer() {
    //RobotContainer() constructor initializing all the button variables to the buttonboard
    m_buttonboardA = new Joystick(0);
    m_buttonboardB = new Joystick(2);
    SmartDashboard.putNumber("Joystick Degree",   2.0);
    m_shooter = new Shooter(58, 36, 35);
    m_drivetrain = new Drivetrain();
    m_joystick = new Joystick(1);
    navxResetButton = new Trigger(() -> m_joystick.getRawButton(3));
    zeroIntake = new Trigger(() -> m_buttonboardB.getRawButton(3));
    flywheelIn = new Trigger(()-> m_buttonboardA.getRawButton(6));
    coastMode = new Trigger(() -> m_buttonboardA.getRawButton(2));
    indexIn = new Trigger(()-> m_buttonboardA.getRawButton(4));
    intakeUpManual = new Trigger(() -> m_buttonboardB.getRawButton(11));
    intakeDownManual = new Trigger(() ->m_buttonboardB.getRawButton(12));
    autoShootTrigger = new Trigger(() ->m_buttonboardA.getRawButton(8));
    shootTrigger = new Trigger(() -> m_buttonboardA.getRawButton(7));
    indexTrigger = new Trigger(() -> m_buttonboardA.getRawButton(5));
    m_intake = new Intake(38, 39, 53, ()-> robotState);
    intakeUp = new Trigger(() -> m_buttonboardB.getRawButton(15));
    intakeDown = new Trigger(() -> m_buttonboardB.getRawButton(16));
    feederIn = new Trigger(()-> m_buttonboardB.getRawButton(9));
    feederOut = new Trigger(() -> m_buttonboardB.getRawButton(10));
    robotState = RobotState.NEUTRAL; //instantiate robotState 
   // trapezoidalTrigger = new Trigger(() -> m_joystick.getRawButton(6));
  

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
                Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(m_drivetrain.distanceToRPM()))), //Sets the power of the shooter
                Commands.run(() -> m_shooter.shooter2Motor.setControl(new VelocityVoltage(m_drivetrain.distanceToRPM() * 4))), //Sets the power of the other shooter motor, multiplies it by 4 due to the gearbox
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.067), //How long to wait before accelerating to the max velocity of the shooter
                    Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)) //Sets to the max voltage
                )
            ).finallyDo((x)->{m_shooter.shooterMotor.set(0); //When the command is done running set the power of both the motors to 0
              m_shooter.shooter2Motor.set(0); 
              m_shooter.indexMotor.set(0);});
    shootCommand.addRequirements(m_shooter); //Add this command to the shooter class

    //Create a name command that is called shoot2, used in our autonomous code
    NamedCommands.registerCommand("Shoot2", 
    new ParallelCommandGroup (
                Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(48.2))), //Sets the power of the shooter motor
                Commands.run(() -> m_shooter.shooter2Motor.setControl(new VelocityVoltage(48.2*4))), //Sets the power of the other shooter motor, multiples it by 4 due to the gearbox
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.067),
                    Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE))
                )
            ).finallyDo((x)->{m_shooter.shooterMotor.set(0); 
              m_shooter.shooter2Motor.set(0); 
              m_shooter.indexMotor.set(0);}));

    
    //Runs the index motor, and when it is done running slow it down until it reaches 0
    NamedCommands.registerCommand("Index", Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)).finallyDo(() -> m_shooter.indexMotor.setVoltage(0)));

    //Using motion magic to control the voltage of intake motor, depending on the position the intake is at. Also controls the feedwheel power depending on the position of intake (when it is at rest).
    NamedCommands.registerCommand("Intake", Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheelL.set(Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0));
      m_intake.feederWheelR.set(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?-1 * Constants.MAX_FLYWHEEL_VOLTAGE:0); //Sets the feederwheel power, to the opposite of the left because this one is reversed
    }));

    //trapezoidalCommand = new Trapezoidal(m_drivetrain,3,3,2);
    //autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
  }
  
  //Returns the autonomous command you want to run in the match
  public Command getAutonomousCommand(){
    //String x = autoChooser.getSelected().getName();

    /* 
    try{
      return new PathPlannerAuto("opus");
    } catch (Exception e){
      System.out.print("Caught the pahtlplanner lau");
    }
    finally{
      System.out.print("entered try catch");
    }
    return null;
    */

    SmartDashboard.putString("auto working", "yes");
    return new PathPlannerAuto("opus");
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(m_driveCommand); //Set the default drivecommand to drivetrain
    //trapezoidalTrigger.whileTrue(trapezoidalCommand);

    //While coastMode is true, set the value fo intakeMotor to coast otherwise, set the state of the intake motor to BRAKE
    coastMode.whileTrue(Commands.run(()-> {
      m_intake.intakeMotor.setNeutralMode(NeutralModeValue.Coast);
      m_intake.intakeMotor.set(0);
    }));
    coastMode.onFalse(Commands.run(() -> {
      m_intake.intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }));

    //If the zeroIntake button is pressed, set the intake position to 0
    zeroIntake.onTrue(Commands.runOnce(() -> {
      m_intake.intakeMotor.setPosition(0);
    }));
   
    //While the intake is up, set the power of the feedwheels to 0
    intakeUp.whileTrue(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_UP));
      m_intake.feederWheelL.set(0);
      m_intake.feederWheelR.set(0);
    }));

    //While the intake is not up or down, set both the feederwheels to 0
    intakeUp.or(intakeDown).whileFalse(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(m_intake.intakeMotor.getPosition().getValueAsDouble()));
      m_intake.feederWheelL.set(0);
      m_intake.feederWheelR.set(0);
    }, m_intake));

    //While the intake is down, set the voltage of the feedwheels to their max voltage
    intakeDown.whileTrue(Commands.run(()-> {
      m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION));
      m_intake.feederWheelL.set(Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7? Constants.MAX_FLYWHEEL_VOLTAGE:0));
      m_intake.feederWheelR.set(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7? -1 * Constants.MAX_FLYWHEEL_VOLTAGE:0);
      //m_intake.feederWheelL.set((Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0)));
      //m_intake.feederWheelR.set((Math.abs(m_intake.intakeMotor.getPosition().getValueAsDouble()-Constants.INTAKE_DOWN_POSITION<.7?Constants.MAX_FLYWHEEL_VOLTAGE:0)));
    }, m_intake));

    //While you want the intake to run the feedwheels inside, set the direction of fly wheel voltages to normal
    feederIn.whileTrue(Commands.run(()-> {
      m_intake.feederWheelL.set(Constants.MAX_FLYWHEEL_VOLTAGE); 
      m_intake.feederWheelR.set(-1 * Constants.MAX_FLYWHEEL_VOLTAGE);
    }));

    //When you dont want the feedwheels to run in, you set the power of both of them to 0
    feederIn.whileFalse(Commands.runOnce(()-> {
      m_intake.feederWheelL.set(0); 
      m_intake.feederWheelR.set(0);
    }));
    
    //When you want the feederwheels to run the opposite way, away from the robot you set them to the their negative direction
    feederOut.whileTrue(Commands.run(()-> {
      m_intake.feederWheelL.set(-1 * Constants.MAX_FLYWHEEL_VOLTAGE); 
      m_intake.feederWheelR.set(Constants.MAX_FLYWHEEL_VOLTAGE);
    }));

    //While this is false, you set the powers of both of them to 0
    feederOut.whileFalse(Commands.runOnce(()->{
      m_intake.feederWheelL.set(0); 
      m_intake.feederWheelR.set(0);
    }));
    
    //When the flywheels run in set the power of the motor to their reversed values
    flywheelIn.whileTrue(Commands.run(()->{
      m_shooter.shooterMotor.set(-1 * Constants.MAX_FLYWHEEL_VOLTAGE); 
      m_shooter.shooter2Motor.set(-1 * Constants.MAX_FLYWHEEL_VOLTAGE*4);
    }));

    //When this is false, set the values of the motor to 0
    flywheelIn.onFalse(Commands.runOnce(()->{
      m_shooter.shooterMotor.set(0); 
      m_shooter.shooter2Motor.set(0);
    }));

    indexIn.whileTrue(Commands.run(()->m_shooter.indexMotor.set(-1 * Constants.MAX_INDEX_VOLTAGE)));
    indexIn.onFalse(Commands.runOnce(()->m_shooter.indexMotor.set(0)));
  

    intakeUpManual.whileTrue(Commands.run(()-> m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(0))));  //Brings the intake up
    intakeDownManual.whileTrue(Commands.run(()->m_intake.intakeMotor.setControl(new MotionMagicDutyCycle(Constants.INTAKE_DOWN_POSITION)))); //Bring the intake down


    autoShootTrigger.whileTrue(shootCommand); //While the autoshoot trigger is pressed, run the shoot command

    indexTrigger.whileTrue(Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE)));
    indexTrigger.onFalse(Commands.runOnce(() -> m_shooter.indexMotor.setVoltage(0)));


    //Manual shooting (MOMO), run to the front of the hub and staet shooting
    shootTrigger.whileTrue( new ParallelCommandGroup (
                Commands.run(() -> m_shooter.shooterMotor.setControl(new VelocityVoltage(48.2))),
                Commands.run(() -> m_shooter.shooter2Motor.setControl(new VelocityVoltage(48.2*4))),
                new SequentialCommandGroup(
                    Commands.waitSeconds(1.067),//  TEST TS
                    Commands.run(() -> m_shooter.indexMotor.setVoltage(Constants.MAX_INDEX_VOLTAGE))
                )
            ).finallyDo((x)->{m_shooter.shooterMotor.set(0); 
              m_shooter.shooter2Motor.set(0); 
              m_shooter.indexMotor.set(0);}));
              
  navxResetButton.onTrue(Commands.runOnce(m_drivetrain::zeroGyro)); //Reset the IMU (NAVX) of the robot


  }
  

  public void resetPose(){
    m_drivetrain.swerveDrive.resetOdometry(new Pose2d());
  }

  public void periodic(){
  }
  


  //Class containing the states of the robot
  public enum RobotState{
    NEUTRAL,
    INTAKE,
    OUTTAKE,
    SHOOT;  
  }
}

