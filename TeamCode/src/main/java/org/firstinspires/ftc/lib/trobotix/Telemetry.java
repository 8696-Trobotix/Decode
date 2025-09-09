// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public final class Telemetry {
  private static org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

  private static TelemetryPacket dashboardTelemetryPacket = new TelemetryPacket(false);
  private static final FtcDashboard dashboard = FtcDashboard.getInstance();

  static void setTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
    Telemetry.telemetry = telemetry;
  }

  static void initTelemetry() {
    telemetry.setMsTransmissionInterval(20);
    dashboard.setTelemetryTransmissionInterval(20);
  }

  static void sendDashboardTelemetry() {
    dashboard.sendTelemetryPacket(dashboardTelemetryPacket);
    dashboardTelemetryPacket = new TelemetryPacket();
  }

  public static void addDSLine(String line) {
    if (telemetry == null) {
      return;
    }
    telemetry.addLine(line);
  }

  public static void addDSData(String name, Object data) {
    if (telemetry == null) {
      return;
    }
    telemetry.addData(name, data);
    dashboardTelemetryPacket.put("DS/" + name, data);
  }

  public static void addDashboardData(String path, Object data) {
    if (data instanceof Rotation2d rotation2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Rotation2d");
      dashboardTelemetryPacket.put(path + "/value", rotation2d.getRadians());
    } else if (data instanceof Translation2d translation2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation2d");
      dashboardTelemetryPacket.put(path + "/x", translation2d.getX());
      dashboardTelemetryPacket.put(path + "/y", translation2d.getY());
    } else if (data instanceof Pose2d pose2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Pose2d");
      addDashboardData(path + "/translation", pose2d.getTranslation());
      addDashboardData(path + "/rotation", pose2d.getRotation());
    } else if (data instanceof ChassisSpeeds chassisSpeeds) {
      dashboardTelemetryPacket.put(path + "/.type", "ChassisSpeeds");
      dashboardTelemetryPacket.put(path + "/vx", chassisSpeeds.vxMetersPerSecond);
      dashboardTelemetryPacket.put(path + "/vy", chassisSpeeds.vyMetersPerSecond);
      dashboardTelemetryPacket.put(path + "/omega", chassisSpeeds.omegaRadiansPerSecond);
    } else {
      dashboardTelemetryPacket.put(path, data);
    }
  }
}
