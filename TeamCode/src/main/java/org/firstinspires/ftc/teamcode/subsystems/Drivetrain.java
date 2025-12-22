// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.Pinpoint;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.Commands;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.filter.SlewRateLimiter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
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

  private final PIDController xPid;
  private final PIDController yPid;
  private final PIDController yawPid;
  private final Pinpoint pinpoint;

  public Drivetrain() {
    frontLeft = new Motor("Motor3");
    frontRight = new Motor("Motor2");
    backLeft = new Motor("Motor1");
    backRight = new Motor("Motor0");

    pinpoint = new Pinpoint("odo", 0.004, -0.004, false, false);

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

    xPid = new PIDController(5, 0, 0);
    yPid = new PIDController(5, 0, 0);
    yawPid = new PIDController(5, 0, 0);
    distancePid = new PIDController(5, 0, 0);
    xPid.setTolerance(.1, .5);
    yPid.setTolerance(.1, .5);
    distancePid.setTolerance(.1, .5);
    yawPid.setTolerance(.25, .75);
    yawPid.enableContinuousInput(-Math.PI, Math.PI);
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

  @Override
  public void periodic() {
    Telemetry.addDashboardData("Pinpoint pose", pinpoint.getFreshPose());
  }

  private boolean onRed = false;

  public Command setOnBlue() {
    return Commands.runOnce(() -> onRed = false);
  }

  public Command setOnRed() {
    return Commands.runOnce(() -> onRed = true);
  }

  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier omegaInput) {
    return fieldRelativeDrive(
        () ->
            new ChassisSpeeds(
                xInput.getAsDouble() * maxSpeedMetersPerSec * (onRed ? -1 : 1),
                yInput.getAsDouble() * maxSpeedMetersPerSec * (onRed ? -1 : 1),
                omegaInput.getAsDouble() * maxAngularSpeedRadPerSec));
  }

  public Command fieldRelativeDrive(Supplier<ChassisSpeeds> speeds) {
    return robotRelativeDrive(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.get(), pinpoint.getCachedPose().getRotation()));
  }

  public Command driveToPose(Pose2d targetPose) {
    return fieldRelativeDrive(
            () -> {
              var currentPose = pinpoint.getCachedPose();
              return new ChassisSpeeds(
                  xPid.calculate(currentPose.getX(), targetPose.getX()),
                  yPid.calculate(currentPose.getY(), targetPose.getY()),
                  yawPid.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians()));
            })
        .until(
            () -> {
              var delta = pinpoint.getCachedPose().minus(targetPose);
              return delta.getTranslation().getNorm() < .05
                  && Math.abs(delta.getRotation().getDegrees()) < 5;
            });
  }

  private final PIDController distancePid;
  private final double targetDistanceMeters = .5;

  public Command aimAtGoal(DoubleSupplier strafeInput) {
    var redGoal = new Translation2d(0, 0);
    var blueGoal = new Translation2d(Units.feetToMeters(12) - redGoal.getX(), redGoal.getY());
    return fieldRelativeDrive(
        () -> {
          var goal = onRed ? redGoal : blueGoal;
          var currentPose = pinpoint.getCachedPose();
          var currentPoseToGoalDelta = goal.minus(currentPose.getTranslation());
          var currentPoseToGoalAngle = currentPoseToGoalDelta.getAngle();
          var distanceControl =
              distancePid.calculate(currentPoseToGoalDelta.getNorm(), targetDistanceMeters);
          var strafeControl = strafeInput.getAsDouble() * maxSpeedMetersPerSec / 2;
          var strafeDirection = currentPoseToGoalAngle.plus(Rotation2d.kCCW_90deg);
          return new ChassisSpeeds(
              distanceControl * currentPoseToGoalAngle.getCos()
                  + strafeControl * strafeDirection.getCos(),
              distanceControl * currentPoseToGoalAngle.getSin()
                  + strafeControl * strafeDirection.getSin(),
              yawPid.calculate(
                      currentPose.getRotation().getRadians(), currentPoseToGoalAngle.getRadians())
                  - strafeControl / currentPoseToGoalDelta.getNorm());
        });
  }

  public boolean atTargetDistance() {
    return distancePid.atSetpoint() && yawPid.atSetpoint();
  }

  public Command robotRelativeDrive(Supplier<ChassisSpeeds> speeds) {
    return run(() -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds.get());
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

  public Command setPose(Pose2d pose) {
    return Commands.runOnce(() -> pinpoint.resetPose(pose));
  }

  public Command resetGyro() {
    return setGyro(Rotation2d.kZero);
  }

  public Command setGyro(Rotation2d newYaw) {
    return Commands.runOnce(() -> pinpoint.resetRotation(newYaw));
  }
}
