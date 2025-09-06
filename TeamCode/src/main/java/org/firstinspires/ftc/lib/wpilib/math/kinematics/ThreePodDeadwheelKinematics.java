// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.kinematics;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Twist2d;

public class ThreePodDeadwheelKinematics
    implements Kinematics<ThreePodDeadwheelSpeeds, ThreePodDeadwheelPositions> {
  private final double x0WheelYPos; // Y-position of one of the encoders measuring the x direction;
  private final double x1WheelYPos; // Y-position of the other encoders measuring the x direction;
  private final double yWheelXPos; // X-position of the encoder measuring the y direction;

  /**
   * Constructs a two pod deadwheel kinematics object.
   *
   * @param x0WheelYPos The Y-position of one of the encoders measuring the x direction. (The
   *     left-right position of the deadwheel measuring the forward-back direction, left+)
   * @param x1WheelYPos The Y-position of the other encoder measuring the x direction. (The
   *     left-right position of the deadwheel measuring the forward-back direction, left+)
   * @param yWheelXPos The X-position of the encoder measuring the y direction. (The forward-back
   *     position of the deadwheel measuring the left-right direction, forward+)
   */
  public ThreePodDeadwheelKinematics(double x0WheelYPos, double x1WheelYPos, double yWheelXPos) {
    this.x0WheelYPos = x0WheelYPos;
    this.x1WheelYPos = x1WheelYPos;
    this.yWheelXPos = yWheelXPos;
  }

  @Override
  public ChassisSpeeds toChassisSpeeds(ThreePodDeadwheelSpeeds wheelSpeeds) {
    return new ChassisSpeeds(
        (wheelSpeeds.x0VelMetersPerSec * x1WheelYPos - wheelSpeeds.x1VelMetersPerSec * x0WheelYPos)
            / (x1WheelYPos - x0WheelYPos),
        wheelSpeeds.yVelMetersPerSec
            - yWheelXPos
                * (wheelSpeeds.x0VelMetersPerSec - wheelSpeeds.x1VelMetersPerSec)
                / (x1WheelYPos - x0WheelYPos),
        (wheelSpeeds.x0VelMetersPerSec - wheelSpeeds.x1VelMetersPerSec)
            / (x1WheelYPos - x0WheelYPos));
  }

  @Override
  public ThreePodDeadwheelSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return new ThreePodDeadwheelSpeeds(
        chassisSpeeds.vxMetersPerSecond - x0WheelYPos * chassisSpeeds.omegaRadiansPerSecond,
        chassisSpeeds.vxMetersPerSecond - x1WheelYPos * chassisSpeeds.omegaRadiansPerSecond,
        chassisSpeeds.vyMetersPerSecond + yWheelXPos * chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public Twist2d toTwist2d(ThreePodDeadwheelPositions start, ThreePodDeadwheelPositions end) {
    var deltaX0 = end.x0PosMeters - start.x0PosMeters;
    var deltaX1 = end.x1PosMeters - start.x1PosMeters;
    var deltaY = end.yPosMeters - start.yPosMeters;
    return new Twist2d(
        (deltaX0 * x1WheelYPos - deltaX1 * x0WheelYPos) / (x1WheelYPos - x0WheelYPos),
        deltaY - yWheelXPos * (deltaX0 - deltaX1) / (x1WheelYPos - x0WheelYPos),
        (deltaX0 - deltaX1) / (x1WheelYPos - x0WheelYPos));
  }

  @Override
  public ThreePodDeadwheelPositions copy(ThreePodDeadwheelPositions positions) {
    return new ThreePodDeadwheelPositions(
        positions.x0PosMeters, positions.x1PosMeters, positions.yPosMeters);
  }

  @Override
  public void copyInto(ThreePodDeadwheelPositions positions, ThreePodDeadwheelPositions output) {
    output.x0PosMeters = positions.x0PosMeters;
    output.x1PosMeters = positions.x1PosMeters;
    output.yPosMeters = positions.yPosMeters;
  }

  @Override
  public ThreePodDeadwheelPositions interpolate(
      ThreePodDeadwheelPositions startValue, ThreePodDeadwheelPositions endValue, double t) {
    return startValue.interpolate(endValue, t);
  }
}
