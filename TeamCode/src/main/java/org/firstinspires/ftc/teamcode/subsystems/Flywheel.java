// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final Motor motor = new Motor("Motor4");

  public Command spinUp() {
    return run(
        () -> {
          double targetRPM = 5000;
          var feedforward = targetRPM * (12.0 / 6000);
          motor.setVoltage(feedforward);
        });
  }
}
