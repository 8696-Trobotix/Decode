// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

public class ThreePodDeadwheelSpeeds {
  public double x0VelMetersPerSec;
  public double x1VelMetersPerSec;
  public double yVelMetersPerSec;

  public ThreePodDeadwheelSpeeds(
      double x0VelMetersPerSec, double x1VelMetersPerSec, double yVelMetersPerSec) {
    this.x0VelMetersPerSec = x0VelMetersPerSec;
    this.x1VelMetersPerSec = x1VelMetersPerSec;
    this.yVelMetersPerSec = yVelMetersPerSec;
  }
}
