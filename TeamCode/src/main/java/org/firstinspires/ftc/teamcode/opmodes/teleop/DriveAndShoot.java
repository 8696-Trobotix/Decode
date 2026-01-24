// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.parallel;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;

@TeleOp
public class DriveAndShoot extends BaseOpMode {
  @Override
  protected void initialize() {
    drivetrain.setDefaultCommand(
        drivetrain.teleopDrive(
            () -> -primaryController.getLeftY(),
            () -> -primaryController.getLeftX(),
            () -> -primaryController.getRightX()));
    primaryController.a().onTrue(drivetrain.resetGyro());
    primaryController
        .leftTrigger()
        .whileTrue(drivetrain.aimAtGoal(() -> -primaryController.getLeftX()));
    primaryController
        .rightTrigger()
        .whileTrue(parallel(feeder.feed().repeatedly(), flywheel.spinUp()));
    primaryController.b().whileTrue(feeder.unfeed());
  }
}
