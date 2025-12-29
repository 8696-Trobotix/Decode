// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.estimator.PinpointPoseEstimator;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.Pinpoint;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.Commands;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveWheelSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Drivetrain extends SubsystemBase {
  private final Motor frontLeft, frontRight, backLeft, backRight;

  private final PIDController xPid;
  private final PIDController yPid;
  private final PIDController yawPid;
  private final PinpointPoseEstimator poseEstimator;
  private final AprilTagProcessor tagProcessor;

  public Drivetrain() {
    frontLeft = new Motor("Motor3");
    frontRight = new Motor("Motor2");
    backLeft = new Motor("Motor1");
    backRight = new Motor("Motor0");

    poseEstimator =
        new PinpointPoseEstimator(
            new Pinpoint("odo", 0.004, -0.004, true, false),
            VecBuilder.fill(.1, .1, .1),
            VecBuilder.fill(.9, .9, .9));
    var cameraPose = Transform3d.kZero;
    tagProcessor =
        new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
            .setCameraPose(
                CoordinateSystems.WPILibToRobotCoordinates(cameraPose.getTranslation()),
                CoordinateSystems.WPILibToSDKRotation(
                    cameraPose.getRotation().plus(new Rotation3d(0, Math.PI / 2, 0))))
            .setLensIntrinsics(
                // TODO: Measure actual values
                639.5 / Math.tan(Units.degreesToRadians(35)),
                639.5 / Math.tan(Units.degreesToRadians(35)),
                639.5,
                399.5)
            .setDrawCubeProjection(true)
            .setDrawAxes(true)
            .setDrawTagOutline(true)
            .build();
    tagProcessor.setDecimation(2);
    new VisionPortal.Builder()
        .setCamera(BaseOpMode.hardwareMap.get(WebcamName.class, "camera"))
        .setCameraResolution(new Size(1280, 800))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .addProcessor(tagProcessor)
        .enableLiveView(false)
        .build();

    frontRight.setInverted(true);
    backRight.setInverted(true);
    frontLeft.setBrake(true);
    frontRight.setBrake(true);
    backLeft.setBrake(true);
    backRight.setBrake(true);

    xPid = new PIDController(5, 0, 0.5);
    yPid = new PIDController(5, 0, 0.5);
    yawPid = new PIDController(4, 0, 0.1);
    distancePid = new PIDController(5, 0, 0.5);
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

  private enum Motif {
    GPP,
    PGP,
    PPG
  }

  private Motif motif;

  @Override
  public void periodic() {
    poseEstimator.update();
    var tags = tagProcessor.getDetections();
    for (int i = 0; i < tags.size(); i++) {
      var tag = tags.get(i);
      switch (tag.id) {
        case 21 -> motif = Motif.GPP;
        case 22 -> motif = Motif.PGP;
        case 23 -> motif = Motif.PPG;
        default -> {
          if (tag.robotPose != null) {
            var robotPose = CoordinateSystems.fieldPoseToWPILib(tag.robotPose);
            poseEstimator.addVisionMeasurement(
                robotPose.toPose2d(), tag.frameAcquisitionNanoTime / 1e9);
            Telemetry.addDashboardData("AprilTags/detections/" + i + "/robotPose", robotPose);
          }
        }
      }
      Telemetry.addDashboardData("AprilTags/detections/" + i + "/id", tag.id);
    }
    Telemetry.addDashboardData("Drivetrain/Pinpoint pose", poseEstimator.getEstimatedPosition());
    Telemetry.addDSData("Detected Motif", motif == null ? "None" : motif.name());
    Telemetry.addDashboardData("ZeroPose", Pose3d.kZero);
  }

  private boolean onRed = false;

  public void setOnBlue() {
    onRed = false;
  }

  public void setOnRed() {
    onRed = true;
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
                speeds.get(), poseEstimator.getEstimatedPosition().getRotation()));
  }

  public Command driveToPose(Pose2d targetPose) {
    return fieldRelativeDrive(
            () -> {
              var currentPose = poseEstimator.getEstimatedPosition();
              return new ChassisSpeeds(
                  xPid.calculate(currentPose.getX(), targetPose.getX()),
                  yPid.calculate(currentPose.getY(), targetPose.getY()),
                  yawPid.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians()));
            })
        .until(() -> xPid.atSetpoint() && yPid.atSetpoint() && yawPid.atSetpoint())
        .finallyDo(
            () -> {
              frontLeft.setVoltage(0);
              frontRight.setVoltage(0);
              backLeft.setVoltage(0);
              backRight.setVoltage(0);
            });
  }

  private final PIDController distancePid;
  private final double targetDistanceMeters = 1.75;

  public Command aimAtGoal(DoubleSupplier strafeInput) {
    var redGoal = new Translation2d(0, 0);
    var blueGoal = new Translation2d(Units.feetToMeters(12) - redGoal.getX(), redGoal.getY());
    return fieldRelativeDrive(
        () -> {
          var goal = onRed ? redGoal : blueGoal;
          Telemetry.addDashboardData("Drivetrain/AutoAim/Goal", goal);
          var currentPose = poseEstimator.getEstimatedPosition();
          var currentPoseToGoalDelta = goal.minus(currentPose.getTranslation());
          var currentPoseToGoalAngle = currentPoseToGoalDelta.getAngle();
          var distanceControl =
              -distancePid.calculate(currentPoseToGoalDelta.getNorm(), targetDistanceMeters);
          Telemetry.addDashboardData(
              "Drivetrain/AutoAim/distance", currentPoseToGoalDelta.getNorm());
          Telemetry.addDashboardData("Drivetrain/AutoAim/distanceControl", distanceControl);
          var strafeControl =
              new Translation2d(
                      0,
                      strafeInput.getAsDouble()
                          * maxSpeedMetersPerSec
                          / (2 * currentPoseToGoalDelta.getNorm()))
                  .rotateBy(currentPoseToGoalAngle);
          return new ChassisSpeeds(
              distanceControl * currentPoseToGoalAngle.getCos() + strafeControl.getX(),
              distanceControl * currentPoseToGoalAngle.getSin() + strafeControl.getY(),
              yawPid.calculate(
                      currentPose.getRotation().getRadians(), currentPoseToGoalAngle.getRadians())
                  - strafeControl.getNorm() / currentPoseToGoalDelta.getNorm());
        });
  }

  public boolean atTargetDistance() {
    return distancePid.atSetpoint() && yawPid.atSetpoint();
  }

  public Command robotRelativeDrive(Supplier<ChassisSpeeds> speeds) {
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

  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public Command resetGyro() {
    return setGyro(Rotation2d.kZero);
  }

  public Command setGyro(Rotation2d newYaw) {
    return Commands.runOnce(() -> poseEstimator.resetRotation(newYaw));
  }
}
