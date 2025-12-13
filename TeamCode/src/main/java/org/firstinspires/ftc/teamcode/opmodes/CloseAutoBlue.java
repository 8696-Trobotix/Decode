// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.sequence;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.waitSeconds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class CloseAutoBlue extends BaseOpMode {
  @Override
  protected void initialize() {
    var drivetrain = Robot.getInstance().drivetrain;
    var feeder = Robot.getInstance().feeder;
    enabled()
        .onTrue(
            sequence(
                    drivetrain.setGyro(Rotation2d.fromDegrees(135)),
                    drivetrain.driveRobotRelative(new ChassisSpeeds(.5, 0, 0)).withTimeout(3),
                    feeder.feed(),
                    feeder.feed(),
                    feeder.feed())
                .deadlineFor(Robot.getInstance().flywheel.spinUp())
                .andThen(
                    waitSeconds(1),
                    drivetrain.driveRobotRelative(new ChassisSpeeds(0, -1, 0)).withTimeout(.5)));
  }
}
