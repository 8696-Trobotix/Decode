// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.psilynx.psikit.ftc.GoBildaPinpointDriver;

public class Pinpoint {
  private final GoBildaPinpointDriver pinpoint;

  public Pinpoint(
      String name,
      double xWheelYPosMeters,
      double yWheelXPosMeters,
      boolean xInverted,
      boolean yInverted) {
    pinpoint = BaseOpMode.hardwareMap.get(GoBildaPinpointDriver.class, name);
    pinpoint.setOffsets(xWheelYPosMeters, yWheelXPosMeters, DistanceUnit.METER);
    pinpoint.setEncoderDirections(
        xInverted
            ? GoBildaPinpointDriver.EncoderDirection.FORWARD
            : GoBildaPinpointDriver.EncoderDirection.REVERSED,
        yInverted
            ? GoBildaPinpointDriver.EncoderDirection.FORWARD
            : GoBildaPinpointDriver.EncoderDirection.REVERSED);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setBulkReadScope(
        GoBildaPinpointDriver.Register.X_POSITION,
        GoBildaPinpointDriver.Register.Y_POSITION,
        GoBildaPinpointDriver.Register.H_ORIENTATION);
  }

  private Pose2d cachedPose = new Pose2d();

  public Pose2d getPose(boolean update) {
    if (update) {
      pinpoint.update();
      cachedPose =
          new Pose2d(
              pinpoint.getPosX(DistanceUnit.METER),
              pinpoint.getPosY(DistanceUnit.METER),
              new Rotation2d(pinpoint.getHeading(AngleUnit.RADIANS)));
    }
    return cachedPose;
  }

  public void resetPose(Pose2d pose) {
    pinpoint.setPosition(
        new Pose2D(
            DistanceUnit.METER,
            pose.getX(),
            pose.getY(),
            AngleUnit.RADIANS,
            pose.getRotation().getRadians()));
  }

  public void resetTranslation(Translation2d translation) {
    pinpoint.setPosX(translation.getX(), DistanceUnit.METER);
    pinpoint.setPosY(translation.getY(), DistanceUnit.METER);
  }

  public void resetRotation(Rotation2d rotation) {
    pinpoint.setHeading(rotation.getRadians(), AngleUnit.RADIANS);
  }
}
