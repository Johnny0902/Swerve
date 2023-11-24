// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {
  //Enter WeelRadius Here
  private static final double kWheelRadius = 0.0508;
  //What is encoder resolution???
  private static final int kEncoderResolution = 4096;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final AbsoluteEncoder m_drivEncoder; 

  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int driveEncoderChannelA,
    int driveEncoderChannelB,
    int turningEncoderChannelA,
    int turningEncoderChannelB
  ) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
}
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
