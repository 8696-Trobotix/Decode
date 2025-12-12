// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Quaternion;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform3d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagCamera {
  private final AprilTagProcessor processor;
  private final VisionPortal visionPortal;

  private final String name;
  private final AprilTagLibrary tagLibrary;

  public AprilTagCamera(String name, Transform3d cameraPose) {
    tagLibrary = AprilTagGameDatabase.getDecodeTagLibrary();
    this.name = name;
    processor =
        new AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(tagLibrary)
            .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
            .setCameraPose(
                CoordinateSystems.WPILibToFieldCoordinates(cameraPose.getTranslation()),
                CoordinateSystems.WPILibToSDKRotation(cameraPose.getRotation()))
            .build();
    processor.setDecimation(3);

    visionPortal =
        new VisionPortal.Builder()
            .setCamera(BaseOpMode.hardwareMap.get(WebcamName.class, name))
            .addProcessor(processor)
            .build();
  }

  public AprilTag[] getTags() {
    var detections = processor.getDetections();
    var tags = new AprilTag[detections.size()];
    var tagIds = new int[detections.size()];
    var tagPoses = new Pose3d[detections.size()];
    for (int i = 0; i < tags.length; i++) {
      var detection = detections.get(i);
      tags[i] =
          new AprilTag(detection.id, CoordinateSystems.fieldPoseToWPILib(detection.robotPose));
      tagIds[i] = detection.id;
      tagPoses[i] =
          switch (detection.id) {
            case 21, 22, 23 ->
                new Pose3d(0, Units.feetToMeters(6), Units.feetToMeters(1), Rotation3d.kZero);
            default -> {
              var tag = tagLibrary.lookupTag(detection.id);
              yield new Pose3d(
                  CoordinateSystems.fieldCoordinatesToWPILib(
                      new Position(
                          tag.distanceUnit,
                          tag.fieldPosition.get(0),
                          tag.fieldPosition.get(1),
                          tag.fieldPosition.get(2),
                          0)),
                  new Rotation3d(
                      new Quaternion(
                          tag.fieldOrientation.w,
                          tag.fieldOrientation.x,
                          tag.fieldOrientation.y,
                          tag.fieldOrientation.z)));
            }
          };
    }
    Telemetry.addDashboardData(name + "detectedTagIds", tagIds);
    Telemetry.addDashboardData(name + "detectedTagPoses", tagPoses);
    return tags;
  }

  public record AprilTag(int id, Pose3d robotPose) {}

  public void stream() {
    visionPortal.resumeStreaming();
  }

  public void stopStream() {
    visionPortal.stopStreaming();
  }
}
