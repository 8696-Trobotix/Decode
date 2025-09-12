// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.trobotix.hardware.ModeledMotor;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class Drivetrain extends SubsystemBase {
  private final ModeledMotor frontLeft, frontRight, backLeft, backRight;

  public Drivetrain() {
    frontLeft =
        new ModeledMotor(
            new Motor("Motor0"),
            new Encoder("Motor0", Encoder.CountsPerRevolution.GOBILDA_435RPM),
            DCMotor.GOBILDA_5203_435RPM(1),
            6,
            4);
    frontRight =
        new ModeledMotor(
            new Motor("Motor1"),
            new Encoder("Motor1", Encoder.CountsPerRevolution.GOBILDA_435RPM),
            DCMotor.GOBILDA_5203_435RPM(1),
            6,
            4);
    backLeft =
        new ModeledMotor(
            new Motor("Motor2"),
            new Encoder("Motor2", Encoder.CountsPerRevolution.GOBILDA_435RPM),
            DCMotor.GOBILDA_5203_435RPM(1),
            6,
            4);
    backRight =
        new ModeledMotor(
            new Motor("Motor3"),
            new Encoder("Motor3", Encoder.CountsPerRevolution.GOBILDA_435RPM),
            DCMotor.GOBILDA_5203_435RPM(1),
            6,
            4);

    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  private final double wheelbaseLengthMeters = Units.inchesToMeters(10);
  private final double wheelbaseWidthMeters = Units.inchesToMeters(10);
  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2));

  private final double maxSpeedMetersPerSec =
      DCMotor.GOBILDA_5203_435RPM(1).freeSpeedRadPerSec * (52.0 / 1000);
  private final double kV_voltsPerMetersPerSec = 12 / maxSpeedMetersPerSec;

  public Command drive(Supplier<ChassisSpeeds> speeds) {
    return run(
        () -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds.get());
          wheelSpeeds.desaturate(maxSpeedMetersPerSec);

          frontLeft.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.frontLeftMetersPerSecond);
          frontRight.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.frontRightMetersPerSecond);
          backLeft.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.rearLeftMetersPerSecond);
          backRight.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.rearRightMetersPerSecond);
        });
  }
}
