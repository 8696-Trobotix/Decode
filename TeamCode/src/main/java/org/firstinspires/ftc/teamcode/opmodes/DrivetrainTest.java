// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class DrivetrainTest extends BaseOpMode {
  @Override
  protected void initialize() {
    enabled()
        .whileTrue(
            Robot.getInstance()
                .drivetrain
                .drive(() -> new ChassisSpeeds(-primaryController.getLeftY(), 0, 0)));
  }
}
