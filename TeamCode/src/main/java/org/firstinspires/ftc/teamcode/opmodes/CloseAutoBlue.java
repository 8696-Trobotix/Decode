// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.parallel;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.sequence;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.waitSeconds;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.waitUntil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
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
                    drivetrain.setPose(
                        new Pose2d(Units.feetToMeters(12) - .7, .355, Rotation2d.fromDegrees(35))),
                    drivetrain.setOnBlue(),
                    parallel(
                        drivetrain.aimAtGoal(() -> 0),
                        sequence(
                            waitUntil(drivetrain::atTargetDistance),
                            feeder.feed(),
                            feeder.feed(),
                            feeder.feed())))
                .deadlineFor(Robot.getInstance().flywheel.spinUp())
                .andThen(
                    waitSeconds(1),
                    drivetrain.driveToPose(
                        new Pose2d(Units.feetToMeters(12) - 1.25, 1.75, Rotation2d.kCW_90deg))));
  }
}
