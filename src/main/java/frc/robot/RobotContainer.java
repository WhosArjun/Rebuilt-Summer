// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.TrapezoidalEdge;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  
  public RobotState robotState; //robotstate (NUETRAL)
  public final Drivetrain m_drivetrain; //Declare drivetrain
  public final Joystick m_joystick; //Declare joystick
  //Use the joystick values as the doubleSuppliers.
  public final DriveCommand m_driveCommand; //(xTranslation, yTranslation, thetaTranslation)
  public final Trigger trapezoidalTrigger;
  public final TrapezoidalEdge trapezoidalCommand;

  private final SendableChooser<Command> autoChooser;
 
                                                          
  
  public RobotContainer() {
    m_drivetrain = new Drivetrain();
    m_joystick = new Joystick(1);
    trapezoidalTrigger = new Trigger(() -> m_joystick.getRawButton(6));
    robotState = RobotState.NEUTRAL; //instantiate robotState 
    //Drive command constructor, using drivecommand in robot container allows for the drivecommand class to be utilized.
    m_driveCommand = new DriveCommand(m_drivetrain, 
                                      //I don't know how to fix this yanis (note to self for the meeting), the red lines pmo
                                      () -> {return -m_joystick.getRawAxis(1);},
                                      () -> {return -m_joystick.getRawAxis(0);},
                                      () -> {return -m_joystick.getRawAxis(2);}
                                     ); //(xTranslation, yTranslation, thetaSupplier) define DriveCommand
    trapezoidalCommand = new TrapezoidalEdge(m_drivetrain, 3, 3, 2);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("BMB", new PathPlannerAuto("BMB"));
    autoChooser.addOption("BRMB", new PathPlannerAuto("BRMB"));
    SmartDashboard.putData("Autochooser", autoChooser);
    // System.out.println(m_driveCommand.get);
    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.a().whileTrue(m_driveCommand.Command());
    m_drivetrain.setDefaultCommand(m_driveCommand);
    trapezoidalTrigger.whileTrue(trapezoidalCommand);
    
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public enum RobotState{
    NEUTRAL;
  }
}

