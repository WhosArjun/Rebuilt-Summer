// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Trapezoidal;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  
  public RobotState robotState; 
  public final Drivetrain m_drivetrain; 
  public final Joystick m_joystick;
  public final DriveCommand m_driveCommand; 

  public final Trigger trapezoidalTrigger;
  public final Trapezoidal trapezoidalCommand;
               
  private SendableChooser<Command> autoChooser;
  public RobotContainer() {
    SmartDashboard.putNumber("Joystick Degree", 2.0);
    m_drivetrain = new Drivetrain();
    m_joystick = new Joystick(1);
    robotState = RobotState.NEUTRAL; //instantiate robotState 
    trapezoidalTrigger = new Trigger(() -> m_joystick.getRawButton(6));
    m_driveCommand = new DriveCommand(m_drivetrain, 
                                      () -> {return -m_joystick.getRawAxis(1);},
                                      () -> {return -m_joystick.getRawAxis(0);},
                                      () -> {return -m_joystick.getRawAxis(2);}
                                     ); 

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
    trapezoidalTrigger.whileTrue(trapezoidalCommand);
  }
  

  public void resetPose(){
    m_drivetrain.swerveDrive.resetOdometry(new Pose2d());
  }


  public enum RobotState{
    NEUTRAL;
  }
}

