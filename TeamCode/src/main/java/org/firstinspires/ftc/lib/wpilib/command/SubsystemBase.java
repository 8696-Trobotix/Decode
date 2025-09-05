// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.command;

/**
 * A base for subsystems that handles registration in the constructor, and provides a more intuitive
 * method for setting the default command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class SubsystemBase implements Subsystem {
  /** Constructor. Telemetry/log name defaults to the classname. */
  @SuppressWarnings("this-escape")
  public SubsystemBase() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }
}
