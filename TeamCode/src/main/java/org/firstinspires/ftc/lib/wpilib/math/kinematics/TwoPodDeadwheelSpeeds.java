// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

public class TwoPodDeadwheelSpeeds {
  public double xVelMetersPerSec;
  public double yVelMetersPerSec;
  public double omegaRadPerSec;

  public TwoPodDeadwheelSpeeds(
      double xVelMetersPerSec, double yVelMetersPerSec, double omegaRadPerSec) {
    this.xVelMetersPerSec = xVelMetersPerSec;
    this.yVelMetersPerSec = yVelMetersPerSec;
    this.omegaRadPerSec = omegaRadPerSec;
  }
}
