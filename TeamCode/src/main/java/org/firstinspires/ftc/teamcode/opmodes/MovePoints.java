// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class MovePoints extends BaseOpMode {
  @Override
  protected void initialize() {
    enabled()
        .onTrue(
            Robot.getInstance()
                .drivetrain
                .driveRobotRelative(new ChassisSpeeds(.75, 0, 0))
                .withTimeout(1));
  }
}
