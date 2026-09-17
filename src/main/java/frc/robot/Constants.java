// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final double MAX_SPEED = 6.7;
  public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);  

  public static Translation2d redHub = new Translation2d(11.901, 1.481); //red hub
  public static Translation2d blueHub = new Translation2d(11.901, 4.021); //blue hub

  //INTAKE CONSTANTS
  public static final double INTAKE_DOWN_POSITION = -6.3;


  //OUTTAKE CONSTANTS
  public static final double MAX_FLYWHEEL_VOLTAGE = 12;
  public static final double MAX_INDEX_VOLTAGE = 6;

    public static Translation2d redPassUp = new Translation2d(14.2733333333, 6.456);
  public static Translation2d redPassDown = new Translation2d(14.2733333333, 1.614);
  public static Translation2d bluePassUp = new Translation2d(2.26666666667, 6.456);
  public static Translation2d bluePassDown = new Translation2d(2.26666666667, 1.614);


}
