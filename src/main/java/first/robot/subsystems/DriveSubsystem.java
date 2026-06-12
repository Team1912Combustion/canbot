// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.drive.DifferentialDrive;
//import org.wpilib.drive.RobotDriveBase.MotorType;
//
import first.robot.Constants.DriveConstants;

import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.hardware.motor.PWMSparkMax;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.util.sendable.SendableRegistry;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SparkMax leftLeader = new SparkMax(CANBusMap.CAN_S0, DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(CANBusMap.CAN_S0, DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final SparkMax rightLeader = new SparkMax(CANBusMap.CAN_S0, DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(CANBusMap.CAN_S0, DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive drive =
      new DifferentialDrive(leftLeader::setThrottle, rightLeader::setThrottle);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(drive, leftLeader);
    SendableRegistry.addChild(drive, rightLeader);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerMotorConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    leftLeaderMotorConfig.apply(globalConfig).inverted(false);
    leftFollowerMotorConfig.apply(globalConfig).inverted(false).follow(leftLeader);
    rightLeaderMotorConfig.apply(globalConfig).inverted(true);
    rightFollowerMotorConfig.apply(globalConfig).inverted(true).follow(rightLeader);

    leftLeader.configure(leftLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /*
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }
  */

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("leftDistance", leftLeader::getEncoder, null);
    builder.addDoubleProperty("rightDistance", rightEncoder::getDistance, null);
  }
}
