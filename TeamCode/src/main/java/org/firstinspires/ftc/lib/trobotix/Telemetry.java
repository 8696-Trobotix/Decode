// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

public final class Telemetry {
  private static org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

  static void setTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
    Telemetry.telemetry = telemetry;
  }

  public static void addLine(String line) {
    if (telemetry == null) {
      return;
    }
    telemetry.addLine(line);
  }

  public static void addData(String name, Object data) {
    if (telemetry == null) {
      return;
    }
    telemetry.addData(name, data);
  }
}
