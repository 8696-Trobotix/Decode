// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Robot.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;

@TeleOp
public class Drive extends BaseOpMode {
  @Override
  protected void initialize() {
    enabled()
        .whileTrue(
            drivetrain.teleopDrive(
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX()));
    primaryController.a().onTrue(drivetrain.resetGyro());
  }
}
