// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.MathUtil;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.Interpolatable;

public class ThreePodDeadwheelPositions implements Interpolatable<ThreePodDeadwheelPositions> {
  public double x0PosMeters;
  public double x1PosMeters;
  public double yPosMeters;

  public ThreePodDeadwheelPositions(double x0PosMeters, double x1PosMeters, double yPosMeters) {
    this.x0PosMeters = x0PosMeters;
    this.x1PosMeters = x1PosMeters;
    this.yPosMeters = yPosMeters;
  }

  @Override
  public ThreePodDeadwheelPositions interpolate(ThreePodDeadwheelPositions endValue, double t) {
    return new ThreePodDeadwheelPositions(
        MathUtil.interpolate(this.x0PosMeters, endValue.x0PosMeters, t),
        MathUtil.interpolate(this.x1PosMeters, endValue.x1PosMeters, t),
        MathUtil.interpolate(this.yPosMeters, endValue.yPosMeters, t));
  }
}
