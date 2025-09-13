// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

public interface AprilTagDetectorIO {
  class AprilTagDetectorIOInputs implements LoggableInputs {
    public int[] tagIDs = new int[0];
    public Pose3d[] poses = new Pose3d[0];
    public double[] timestamps = new double[0];

    @Override
    public void toLog(LogTable logTable) {
      logTable.put("tagIDs", tagIDs);
      logTable.put("poses", poses);
      logTable.put("timestamps", timestamps);
    }

    @Override
    public void fromLog(LogTable logTable) {
      tagIDs = logTable.get("tagIDs", new int[0]);
      poses = logTable.get("poses", new Pose3d[0]);
      timestamps = logTable.get("timestamps", new double[0]);
    }
  }

  default void updateInputs(AprilTagDetectorIOInputs inputs) {}
}
