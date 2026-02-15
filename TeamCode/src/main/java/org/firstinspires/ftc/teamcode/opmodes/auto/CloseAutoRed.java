// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.lib.wpilib.command.Commands.*;
import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.Motif.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

@Autonomous
public class CloseAutoRed extends BaseOpMode {
  @Override
  protected void initialize() {
    drivetrain.setPose(
        new Pose2d(
            .355 - Units.feetToMeters(6), Units.feetToMeters(6) - .7, Rotation2d.fromDegrees(126)));
    drivetrain.setOnRed();
    enabled()
        .onTrue(
            sequence(
                // drivetrain
                //     .driveToPose(new Pose2d(-1.2, .45, Rotation2d.fromDegrees(-170)))
                //     .until(() -> drivetrain.getMotif() != null)
                //     .withTimeout(10)
                //     .deadlineFor(feeder.unfeed()),
                // either(
                //     none(),
                //     select(
                //         Map.of(
                //             PPG,
                //             none(),
                //             PGP,
                //             flywheel
                //                 .sort()
                //                 .withDeadline(
                //                     sequence(
                //                         waitUntil(flywheel::isAtTargetRPM)
                //                             .deadlineFor(feeder.unfeed()),
                //                         feeder.feed()))
                //                 .andThen(waitSeconds(1)),
                //             GPP,
                //             flywheel
                //                 .sort()
                //                 .withDeadline(
                //                     sequence(
                //                         waitUntil(flywheel::isAtTargetRPM)
                //                             .deadlineFor(feeder.unfeed()),
                //                         feeder.feed(),
                //                         feeder.feed()))
                //                 .andThen(waitSeconds(1))),
                //         drivetrain::getMotif),
                //     () -> drivetrain.getMotif() == null),
                drivetrain.driveToPose(new Pose2d(-1.2, .45, Rotation2d.fromDegrees(90))),
                waitSeconds(2),
                sequence(
                        waitUntil(flywheel::isAtTargetRPM).deadlineFor(feeder.unfeed()),
                        feeder.feed(),
                        waitSeconds(0.5),
                        feeder.feed(),
                        waitSeconds(0.5),
                        feeder.feed())
                    .deadlineFor(
                        drivetrain.aimAtGoalAuto(new Translation2d(-1.2, .45)),
                        flywheel.shootAtGoal())));
  }
}
