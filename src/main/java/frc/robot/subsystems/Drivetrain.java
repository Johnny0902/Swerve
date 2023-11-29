// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  //NEEDS TO CHECK
  public static final boolean kGyroReversed = false;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;


  // x and y values represent the dimension of the robot (half of the dimention)
  // Set the module of swerve at its location in relation to the center of the robot
  private final Translation2d m_frontLeftLocation = new Translation2d(
    Constants.Dimension_Q2_X, Constants.Dimension_Q2_Y);
  private final Translation2d m_frontRightLocation = new Translation2d(
    Constants.Dimension_Q1_X, Constants.Dimension_Q1_Y);
  private final Translation2d m_backLeftLocation = new Translation2d(
    Constants.Dimension_Q3_X, Constants.Dimension_Q3_X);
  private final Translation2d m_backRightLocation = new Translation2d(
    Constants.Dimension_Q4_X, Constants.Dimension_Q4_X);

  //ERROR: 2 ENCODER NEEDED ON EACH Module
  private final SwerveModule m_frontLeft = new SwerveModule(
    Constants.frontLeftModule.driveMotorChannel, 
    Constants.frontLeftModule.turningMotorChannel, 
    Constants.frontLeftModule.absoluteChannel, 
    Constants.frontLeftModule.canCoderDeviceNumber
  );
  private final SwerveModule m_frontRight = new SwerveModule(
    Constants.frontRightModule.driveMotorChannel, 
    Constants.frontRightModule.turningMotorChannel, 
    Constants.frontRightModule.absoluteChannel, 
    Constants.frontRightModule.canCoderDeviceNumber
  );
  private final SwerveModule m_backLeft = new SwerveModule(
    Constants.backLeftModule.driveMotorChannel, 
    Constants.backLeftModule.turningMotorChannel, 
    Constants.backLeftModule.absoluteChannel, 
    Constants.backLeftModule.canCoderDeviceNumber
  );
  private final SwerveModule m_backRight = new SwerveModule(
    Constants.backRightModule.driveMotorChannel, 
    Constants.backRightModule.turningMotorChannel, 
    Constants.backRightModule.absoluteChannel, 
    Constants.backRightModule.canCoderDeviceNumber
  );

  //QUESTION: WhAT DOES GYRO DO? What Gyro do we use
  private final AnalogGyro m_gyro = new AnalogGyro(Constants.gyroChannel);

  //REREAD DOC + MRJOHNSON
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  //REREAD DOC + MRJOHNSON
  private final SwerveDriveOdometry m_odometry =
    new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

  public Drivetrain() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
  }
    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
//changed param double periodSeconds to Rotation2d rotation 2d

public void drive(
  double xSpeed, double ySpeed, double rot, boolean fieldRelative, Rotation2d rotation2d) {
var swerveModuleStates =
    m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                rotation2d));
SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
m_frontLeft.setDesiredState(swerveModuleStates[0]);
m_frontRight.setDesiredState(swerveModuleStates[1]);
m_backLeft.setDesiredState(swerveModuleStates[2]);
m_backRight.setDesiredState(swerveModuleStates[3]);
}


/** Updates the field relative position of the robot. */
public void updateOdometry() {
  m_odometry.update(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
}

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

 /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
  }

}
