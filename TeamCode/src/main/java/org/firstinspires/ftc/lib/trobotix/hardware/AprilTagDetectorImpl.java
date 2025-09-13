// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import android.util.Size;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform3d;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagDetectorImpl implements AprilTagDetectorIO {
  private final AprilTagProcessor processor;

  public AprilTagDetectorImpl(Transform3d cameraPose, double fx, double cx, double fy, double cy) {
    processor =
        new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            .setCameraPose(
                CoordinateSystems.WPILibToRobotCoordinates(cameraPose.getTranslation()),
                CoordinateSystems.WPILibToSDKRotation(cameraPose.getRotation()))
            .setLensIntrinsics(fx, fy, cx, cy)
            .setNumThreads(2)
            .build();
    processor.setDecimation(3);
    processor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_SOLVEPNP_EPNP);
    new VisionPortal.Builder()
        .setCamera(BaseOpMode.hardwareMap.get(WebcamName.class, "Camera"))
        .setCameraResolution(new Size(1280, 800))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .addProcessor(processor)
        .build();
  }

  @Override
  public void updateInputs(AprilTagDetectorIOInputs inputs) {
    var detections = processor.getFreshDetections();
    if (detections == null) {
      inputs.poses = new Pose3d[0];
      inputs.timestamps = new double[0];
    } else {
      inputs.poses = new Pose3d[detections.size()];
      inputs.timestamps = new double[detections.size()];
      for (int i = 0; i < detections.size(); i++) {
        var robotPose = detections.get(i).robotPose;
        inputs.tagIDs[i] = detections.get(i).id;
        inputs.poses[i] =
            new Pose3d(
                CoordinateSystems.fieldCoordinatesToWPILib(robotPose.getPosition()),
                CoordinateSystems.SDKRotationToWPILib(robotPose.getOrientation()));
        inputs.timestamps[i] =
            robotPose.getPosition().acquisitionTime / 1E9 - BaseOpMode.timeOffset;
      }
    }
  }
}
