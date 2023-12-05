// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double maximumSpeed = 3.0;
  public static final double angleGearRatio = 12.8;
  public static final double driveGearRatio = 6.75;
  public static final double pulsePerRotation = 1;
  public static final double wheelDiameter = 0.1; //this is in meters
  public static final boolean headingCorrection = false;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(
    new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag









//_________________________________________________________________________________________________
  public static final double Dimension_Q1_X = 0.381;
  public static final double Dimension_Q1_Y = 0.381;
  public static final double Dimension_Q2_X = 0.381;
  public static final double Dimension_Q2_Y = -0.381;
  public static final double Dimension_Q3_X = -0.381;
  public static final double Dimension_Q3_Y = -0.381;
  public static final double Dimension_Q4_X = -0.381;
  public static final double Dimension_Q4_Y = 0.381;

  public static class frontLeftModule {
    public static final int driveMotorChannel = 0;
    public static final int turningMotorChannel = 1;
    public static final int absoluteChannel = 2;
    public static final int canCoderDeviceNumber = 3;
  }

  public static class frontRightModule {
    public static final int driveMotorChannel = 4;
    public static final int turningMotorChannel = 5;
    public static final int absoluteChannel = 6;
    public static final int canCoderDeviceNumber = 7;
  }

  public static class backLeftModule {
    public static final int driveMotorChannel = 8;
    public static final int turningMotorChannel = 9;
    public static final int absoluteChannel = 10;
    public static final int canCoderDeviceNumber = 11;
  }

  public static class backRightModule {
    public static final int driveMotorChannel = 12;
    public static final int turningMotorChannel = 13;
    public static final int absoluteChannel = 14;
    public static final int canCoderDeviceNumber = 15;
  }

  public static final int gyroChannel = 0;
  

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
