// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.hardware.Gyro;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.Commands;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.filter.SlewRateLimiter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class Drivetrain extends SubsystemBase {
  private final Motor frontLeft, frontRight, backLeft, backRight;
  private final SlewRateLimiter frontLeftLimiter,
      frontRightLimiter,
      backLeftLimiter,
      backRightLimiter;

  private final Gyro gyro;

  public Drivetrain() {
    frontLeft = new Motor("Motor0");
    frontRight = new Motor("Motor1");
    backLeft = new Motor("Motor2");
    backRight = new Motor("Motor3");

    gyro =
        new Gyro(
            "exhubIMU",
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));

    frontRight.setInverted(true);
    backRight.setInverted(true);
    frontLeft.setBrake(true);
    frontRight.setBrake(true);
    backLeft.setBrake(true);
    backRight.setBrake(true);

    var timeToMaxSpeedSec = .125;
    frontLeftLimiter = new SlewRateLimiter(maxSpeedMetersPerSec / timeToMaxSpeedSec);
    frontRightLimiter = new SlewRateLimiter(maxSpeedMetersPerSec / timeToMaxSpeedSec);
    backLeftLimiter = new SlewRateLimiter(maxSpeedMetersPerSec / timeToMaxSpeedSec);
    backRightLimiter = new SlewRateLimiter(maxSpeedMetersPerSec / timeToMaxSpeedSec);
  }

  private static final double wheelbaseLengthMeters = Units.inchesToMeters(11.5);
  private static final double wheelbaseWidthMeters = Units.inchesToMeters(12.5);
  private static final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(wheelbaseLengthMeters / 2, -wheelbaseWidthMeters / 2),
          new Translation2d(-wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(-wheelbaseLengthMeters / 2, -wheelbaseWidthMeters / 2));

  public static final double maxSpeedMetersPerSec =
      DCMotor.GOBILDA_5203_435RPM(1).freeSpeedRadPerSec * (52.0 / 1000);
  public static final double maxAngularSpeedRadPerSec =
      kinematics.toChassisSpeeds(
              new MecanumDriveWheelSpeeds(
                  -maxSpeedMetersPerSec,
                  maxSpeedMetersPerSec,
                  -maxSpeedMetersPerSec,
                  maxSpeedMetersPerSec))
          .omegaRadiansPerSecond;
  private final double kV_voltsPerMetersPerSec = 12 / maxSpeedMetersPerSec;

  public Command drive(Supplier<ChassisSpeeds> speeds) {
    return run(
        () -> {
          var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), gyro.getYaw());

          var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
          wheelSpeeds.desaturate(maxSpeedMetersPerSec);

          frontLeft.setVoltage(
              kV_voltsPerMetersPerSec
                  * frontLeftLimiter.calculate(wheelSpeeds.frontLeftMetersPerSecond));
          frontRight.setVoltage(
              kV_voltsPerMetersPerSec
                  * frontRightLimiter.calculate(wheelSpeeds.frontRightMetersPerSecond));
          backLeft.setVoltage(
              kV_voltsPerMetersPerSec
                  * backLeftLimiter.calculate(wheelSpeeds.rearLeftMetersPerSecond));
          backRight.setVoltage(
              kV_voltsPerMetersPerSec
                  * backRightLimiter.calculate(wheelSpeeds.rearRightMetersPerSecond));
        });
  }

  public Command driveRobotRelative(ChassisSpeeds speeds) {
    return run(() -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
          wheelSpeeds.desaturate(maxSpeedMetersPerSec);

          frontLeft.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.frontLeftMetersPerSecond);
          frontRight.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.frontRightMetersPerSecond);
          backLeft.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.rearLeftMetersPerSecond);
          backRight.setVoltage(kV_voltsPerMetersPerSec * wheelSpeeds.rearRightMetersPerSecond);
        })
        .finallyDo(
            () -> {
              frontLeft.setVoltage(0);
              frontRight.setVoltage(0);
              backLeft.setVoltage(0);
              backRight.setVoltage(0);
            });
  }

  public Command resetGyro() {
    return setGyro(Rotation2d.kZero);
  }

  public Command setGyro(Rotation2d newYaw) {
    return Commands.runOnce(() -> gyro.setYaw(newYaw));
  }
}
