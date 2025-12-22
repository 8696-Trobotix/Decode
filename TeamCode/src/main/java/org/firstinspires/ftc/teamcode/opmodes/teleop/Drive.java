// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class Drive extends BaseOpMode {
  @Override
  protected void initialize() {
    enabled()
        .whileTrue(
            Robot.getInstance()
                .drivetrain
                .teleopDrive(
                    () -> -primaryController.getLeftY(),
                    () -> -primaryController.getLeftX(),
                    () -> -primaryController.getRightX()));
    primaryController.a().onTrue(Robot.getInstance().drivetrain.resetGyro());
  }
}
