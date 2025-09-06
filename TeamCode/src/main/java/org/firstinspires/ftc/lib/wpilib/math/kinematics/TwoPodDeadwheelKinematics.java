// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;

public class TwoPodDeadwheelKinematics
    implements Kinematics<TwoPodDeadwheelSpeeds, TwoPodDeadwheelPositions> {
  private final double xWheelYPos; // Y-position of the encoder measuring the x direction;
  private final double yWheelXPos; // X-position of the encoder measuring the y direction;

  /**
   * Constructs a two pod deadwheel kinematics object.
   *
   * @param xWheelYPos The Y-position of the encoder measuring the x direction. (The left-right
   *     position of the deadwheel measuring the forward-back direction, left+)
   * @param yWheelXPos The X-position of the encoder measuring the y direction. (The forward-back
   *     position of the deadwheel measuring the left-right direction, forward+)
   */
  public TwoPodDeadwheelKinematics(double xWheelYPos, double yWheelXPos) {
    this.xWheelYPos = xWheelYPos;
    this.yWheelXPos = yWheelXPos;
  }

  @Override
  public ChassisSpeeds toChassisSpeeds(TwoPodDeadwheelSpeeds wheelSpeeds) {
    return new ChassisSpeeds(
        wheelSpeeds.xVelMetersPerSec + xWheelYPos * wheelSpeeds.omegaRadPerSec,
        wheelSpeeds.yVelMetersPerSec - yWheelXPos * wheelSpeeds.omegaRadPerSec,
        wheelSpeeds.omegaRadPerSec);
  }

  @Override
  public TwoPodDeadwheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return new TwoPodDeadwheelSpeeds(
        chassisSpeeds.vxMetersPerSecond - xWheelYPos * chassisSpeeds.omegaRadiansPerSecond,
        chassisSpeeds.vyMetersPerSecond + yWheelXPos * chassisSpeeds.omegaRadiansPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public Twist2d toTwist2d(TwoPodDeadwheelPositions start, TwoPodDeadwheelPositions end) {
    var headingDeltaRad = end.yawPos.minus(start.yawPos).getRadians();
    return new Twist2d(
        (end.xPosMeters - start.xPosMeters) + xWheelYPos * headingDeltaRad,
        (end.yPosMeters - start.yPosMeters) - yWheelXPos * headingDeltaRad,
        headingDeltaRad);
  }

  @Override
  public TwoPodDeadwheelPositions copy(TwoPodDeadwheelPositions positions) {
    return new TwoPodDeadwheelPositions(
        positions.xPosMeters, positions.yPosMeters, positions.yawPos);
  }

  @Override
  public void copyInto(TwoPodDeadwheelPositions positions, TwoPodDeadwheelPositions output) {
    output.xPosMeters = positions.xPosMeters;
    output.yPosMeters = positions.yPosMeters;
    output.yawPos = positions.yawPos;
  }

  @Override
  public TwoPodDeadwheelPositions interpolate(
      TwoPodDeadwheelPositions startValue, TwoPodDeadwheelPositions endValue, double t) {
    return startValue.interpolate(endValue, t);
  }
}
