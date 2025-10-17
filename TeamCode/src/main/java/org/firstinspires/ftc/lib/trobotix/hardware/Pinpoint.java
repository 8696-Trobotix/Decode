// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;

public class Pinpoint {
  private final GoBildaPinpointDriver pinpoint;

  public Pinpoint(
      String name,
      double xWheelYPosMeters,
      double yWheelXPosMeters,
      boolean xInverted,
      boolean yInverted) {
    pinpoint = BaseOpMode.hardwareMap.get(GoBildaPinpointDriver.class, name);
    pinpoint.setOffsets(xWheelYPosMeters, yWheelXPosMeters);
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
      cachedPose = pinpoint.getPose();
    }
    return cachedPose;
  }

  public void resetPose(Pose2d pose) {
    pinpoint.setPose(pose);
  }

  public void resetTranslation(Translation2d translation) {
    pinpoint.setTranslation(translation);
  }

  public void resetRotation(Rotation2d rotation) {
    pinpoint.setHeading(rotation);
  }
}
