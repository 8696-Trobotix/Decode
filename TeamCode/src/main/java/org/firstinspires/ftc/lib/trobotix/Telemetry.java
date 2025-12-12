// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
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

  static void logRobotStats(double dt, double voltage) {
    var memoryUsedMB =
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6;
    dashboardTelemetryPacket.put("Robot/dt", dt);
    dashboardTelemetryPacket.put("Robot/Voltage", voltage);
    dashboardTelemetryPacket.put("Robot/Memory used MB", memoryUsedMB);
    if (telemetry == null) {
      return;
    }
    telemetry.addData("Robot/dt", dt);
    telemetry.addData("Robot/Voltage", voltage);
    telemetry.addData("Robot/Memory used MB", memoryUsedMB);
  }

  public static void addDashboardData(String path, Object data) {
    if (data instanceof Rotation2d rotation2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Rotation2d");
      dashboardTelemetryPacket.put(path + "/value", rotation2d.getRadians());
    } else if (data instanceof Rotation2d[] rotation2ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Rotation2d[]");
      dashboardTelemetryPacket.put(path + "/length", rotation2ds.length);
      for (int i = 0; i < rotation2ds.length; i++) {
        addDashboardData(path + "/" + i, rotation2ds[i]);
      }
    } else if (data instanceof Translation2d translation2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation2d");
      dashboardTelemetryPacket.put(path + "/x", translation2d.getX());
      dashboardTelemetryPacket.put(path + "/y", translation2d.getY());
    } else if (data instanceof Translation2d[] translation2ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation2d[]");
      dashboardTelemetryPacket.put(path + "/length", translation2ds.length);
      for (int i = 0; i < translation2ds.length; i++) {
        addDashboardData(path + "/" + i, translation2ds[i]);
      }
    } else if (data instanceof Pose2d pose2d) {
      dashboardTelemetryPacket.put(path + "/.type", "Pose2d");
      addDashboardData(path + "/translation", pose2d.getTranslation());
      addDashboardData(path + "/rotation", pose2d.getRotation());
    } else if (data instanceof Pose2d[] pose2ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Pose2d[]");
      dashboardTelemetryPacket.put(path + "/length", pose2ds.length);
      for (int i = 0; i < pose2ds.length; i++) {
        addDashboardData(path + "/" + i, pose2ds[i]);
      }
    } else if (data instanceof Translation3d translation3d) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation3d");
      dashboardTelemetryPacket.put(path + "/x", translation3d.getX());
      dashboardTelemetryPacket.put(path + "/y", translation3d.getY());
      dashboardTelemetryPacket.put(path + "/z", translation3d.getZ());
    } else if (data instanceof Translation3d[] translation3ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation3d[]");
      dashboardTelemetryPacket.put(path + "/length", translation3ds.length);
      for (int i = 0; i < translation3ds.length; i++) {
        addDashboardData(path + "/" + i, translation3ds[i]);
      }
    } else if (data instanceof Rotation3d rotation3d) {
      dashboardTelemetryPacket.put(path + "/.type", "Rotation3d");
      dashboardTelemetryPacket.put(path + "/q/.type", "Quaternion");
      dashboardTelemetryPacket.put(path + "/q/w", rotation3d.getQuaternion().getW());
      dashboardTelemetryPacket.put(path + "/q/x", rotation3d.getQuaternion().getX());
      dashboardTelemetryPacket.put(path + "/q/y", rotation3d.getQuaternion().getY());
      dashboardTelemetryPacket.put(path + "/q/z", rotation3d.getQuaternion().getZ());
    } else if (data instanceof Rotation3d[] rotation3ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Translation3d[]");
      dashboardTelemetryPacket.put(path + "/length", rotation3ds.length);
      for (int i = 0; i < rotation3ds.length; i++) {
        addDashboardData(path + "/" + i, rotation3ds[i]);
      }
    } else if (data instanceof Pose3d pose3d) {
      dashboardTelemetryPacket.put(path + "/.type", "Pose3d");
      addDashboardData(path + "/translation", pose3d.getTranslation());
      addDashboardData(path + "/rotation", pose3d.getRotation());
    } else if (data instanceof Pose3d[] pose3ds) {
      dashboardTelemetryPacket.put(path + "/.type", "Pose3d[]");
      dashboardTelemetryPacket.put(path + "/length", pose3ds.length);
      for (int i = 0; i < pose3ds.length; i++) {
        addDashboardData(path + "/" + i, pose3ds[i]);
      }
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
