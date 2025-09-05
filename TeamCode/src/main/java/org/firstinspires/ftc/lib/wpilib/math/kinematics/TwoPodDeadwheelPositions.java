// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.Interpolatable;

public class TwoPodDeadwheelPositions implements Interpolatable<TwoPodDeadwheelPositions> {
  public double xPosMeters;
  public double yPosMeters;
  public Rotation2d yawPos;

  public TwoPodDeadwheelPositions(double xPosMeters, double yPosMeters, Rotation2d yawPos) {
    this.xPosMeters = xPosMeters;
    this.yPosMeters = yPosMeters;
    this.yawPos = yawPos;
  }

  @Override
  public TwoPodDeadwheelPositions interpolate(TwoPodDeadwheelPositions endValue, double t) {
    return new TwoPodDeadwheelPositions(
        MathUtil.interpolate(this.xPosMeters, endValue.xPosMeters, t),
        MathUtil.interpolate(this.yPosMeters, endValue.yPosMeters, t),
        yawPos.interpolate(endValue.yawPos, t));
  }
}
