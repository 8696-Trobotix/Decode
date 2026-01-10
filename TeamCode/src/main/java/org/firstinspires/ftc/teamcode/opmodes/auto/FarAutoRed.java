// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.Robot.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

@Autonomous
public class FarAutoRed extends BaseOpMode {
  @Override
  protected void initialize() {
    drivetrain.setPose(
        new Pose2d(
            Units.feetToMeters(6) - Units.inchesToMeters(9),
            Units.inchesToMeters(8),
            Rotation2d.fromDegrees(90)));
    drivetrain.setOnRed();
    enabled()
        .onTrue(
            drivetrain.driveToPose(
                new Pose2d(
                    Units.feetToMeters(3), Units.inchesToMeters(8), Rotation2d.fromDegrees(180))));
  }
}
