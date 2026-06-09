// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import static org.wpilib.hardware.pneumatic.DoubleSolenoid.Value.FORWARD;
import static org.wpilib.hardware.pneumatic.DoubleSolenoid.Value.REVERSE;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

import first.robot.Constants.CrusherConstants;
import org.wpilib.hardware.pneumatic.DoubleSolenoid;
import org.wpilib.hardware.pneumatic.PneumaticsModuleType;
import org.wpilib.util.sendable.SendableBuilder;

/** A hatch mechanism actuated by a single {@link org.wpilib.hardware.pneumatic.DoubleSolenoid}. */
public class Crusher extends SubsystemBase {

  private final DoubleSolenoid crusherSolenoid =
      new DoubleSolenoid(
          0,
          PneumaticsModuleType.CTRE_PCM,
          CrusherConstants.kCrusherSolenoidPorts[0],
          CrusherConstants.kCrusherSolenoidPorts[1]);

  /** Grabs the hatch. */
  public Command grabHatchCommand() {
    // implicitly require `this`
    return this.runOnce(() -> crusherSolenoid.set(FORWARD));
  }

  /** Releases the hatch. */
  public Command releaseHatchCommand() {
    // implicitly require `this`
    return this.runOnce(() -> crusherSolenoid.set(REVERSE));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the solenoid state to telemetry.
    builder.addBooleanProperty("extended", () -> crusherSolenoid.get() == FORWARD, null);
  }
}
