// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TestTeleop extends BaseOpMode {
  @Override
  protected void initialize() {
    final var drivetrain = Robot.getInstance().drivetrain;

    enabled().whileTrue(drivetrain.drive(new ChassisSpeeds(1, 0, 0)));
  }
}
