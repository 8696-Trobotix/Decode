// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.parallel;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class DriveAndShoot extends BaseOpMode {
  @Override
  protected void initialize() {
    var drivetrain = Robot.getInstance().drivetrain;
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
        .whileTrue(
            parallel(
                Robot.getInstance().feeder.feed().repeatedly(),
                Robot.getInstance().flywheel.spinUp()));
  }
}
