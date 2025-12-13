// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Gyro {
  private final IMU internalIMU;

  public Gyro(String name, ImuOrientationOnRobot orientation) {
    internalIMU = BaseOpMode.hardwareMap.get(IMU.class, name);
    internalIMU.initialize(new IMU.Parameters(orientation));
  }

  private double offsetRad = 0;

  public Rotation2d getYaw() {
    return new Rotation2d(
        internalIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + offsetRad);
  }

  public void setYaw(Rotation2d newYaw) {
    offsetRad =
        newYaw.getRadians() - internalIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
  }
}
