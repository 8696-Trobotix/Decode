// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;

public class TwoPodDeadwheelOdometry extends Odometry<TwoPodDeadwheelPositions> {
  public TwoPodDeadwheelOdometry(
      TwoPodDeadwheelKinematics kinematics,
      Rotation2d gyroAngle,
      double xPosMeters,
      double yPosMeters,
      Pose2d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        new TwoPodDeadwheelPositions(xPosMeters, yPosMeters, gyroAngle),
        initialPoseMeters);
  }

  public TwoPodDeadwheelOdometry(
      TwoPodDeadwheelKinematics kinematics,
      Rotation2d gyroAngle,
      TwoPodDeadwheelPositions wheelPositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
  }

  public Pose2d update(Rotation2d gyroAngle, double xPosMeters, double yPosMeters) {
    return update(gyroAngle, new TwoPodDeadwheelPositions(xPosMeters, yPosMeters, gyroAngle));
  }
}
