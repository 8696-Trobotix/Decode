// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.parallel;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.sequence;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.waitSeconds;
import static org.firstinspires.ftc.lib.wpilib.command.Commands.waitUntil;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

@Autonomous
public class CloseAutoBlue extends BaseOpMode {
  @Override
  protected void initialize() {
    drivetrain.setPose(
        new Pose2d(
            .355 - Units.feetToMeters(6),
            .7 - Units.feetToMeters(6),
            Rotation2d.fromDegrees(-126)));
    drivetrain.setOnBlue();
    enabled()
        .onTrue(
            sequence(
                    waitUntil(drivetrain::atTargetDistance).deadlineFor(feeder.unfeed()),
                    feeder.feed(),
                    feeder.feed(),
                    feeder.feed())
                .deadlineFor(parallel(drivetrain.aimAtGoal(() -> 0), flywheel.spinUp()))
                .andThen(
                    waitSeconds(1),
                    drivetrain.driveToPose(new Pose2d(0.5, -.3, Rotation2d.kCW_90deg))));
  }
}
