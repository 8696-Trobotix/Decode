// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final Servo left = BaseOpMode.hardwareMap.servo.get("Servo0");
  private final Servo right = BaseOpMode.hardwareMap.servo.get("Servo1");

  public Command feed() {
    return startEnd(
        () -> {
          left.setPosition(1);
          right.setPosition(0);
        },
        () -> {
          left.setPosition(0.5);
          right.setPosition(0.5);
        });
  }
}
