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
  private AprilTagProcessor tagProcessor;

  private VisionPortal portal;

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
    var cameraPose =
        new Transform3d(
            -0.1,
            0.1,
            Units.inchesToMeters(17.5),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-69.5),
                Units.degreesToRadians(-90)));
    BaseOpMode.addResetHook(
        () -> {
          tagProcessor =
              new AprilTagProcessor.Builder()
                  .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                  .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                  .setCameraPose(
                      CoordinateSystems.WPILibToRobotCoordinates(cameraPose.getTranslation()),
                      CoordinateSystems.WPILibToSDKRotation(cameraPose.getRotation()))
                  .setLensIntrinsics(
                      911.3052435 * (480.0 / 800),
                      911.5676856 * (480.0 / 800),
                      670.6643003 * (480.0 / 800),
                      430.2322607 * (480.0 / 800))
                  .setDrawCubeProjection(true)
                  .setDrawAxes(true)
                  .setDrawTagOutline(true)
                  .build();
          tagProcessor.setDecimation(2);
          portal =
              new VisionPortal.Builder()
                  .setCamera(BaseOpMode.hardwareMap.get(WebcamName.class, "camera"))
                  .setCameraResolution(new Size(640, 480))
                  .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                  .enableLiveView(true)
                  .addProcessor(tagProcessor)
                  .build();
        });
    BaseOpMode.addCloseHook(
        () -> {
          portal.close();
          portal = null;
          tagProcessor = null;
        });

    frontRight.setInverted(true);
    backRight.setInverted(true);
    frontLeft.setBrake(true);
    frontRight.setBrake(true);
    backLeft.setBrake(true);
    backRight.setBrake(true);

    xPid = new PIDController(5, 0, 0.5);
    yPid = new PIDController(5, 0, 0.5);
    yawPid = new PIDController(4, 0, 0.1);
    xPid.setTolerance(.05, .15);
    yPid.setTolerance(.05, .15);
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

  private final Translation2d redGoal =
      new Translation2d(Units.feetToMeters(-6), Units.feetToMeters(5.75));
  private final Translation2d blueGoal =
      new Translation2d(Units.feetToMeters(-6), Units.feetToMeters(-5.75));

  @Override
  public void periodic() {
    Telemetry.addDSData(
        "Pose estimate yaw", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    Telemetry.addDSData("Camera state", portal.getCameraState().name());
    poseEstimator.update();
    var tags = tagProcessor.getDetections();
    Telemetry.addDSData("Apriltag detections", tags.size());
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
                robotPose.toPose2d(),
                tag.frameAcquisitionNanoTime / 1e9,
                VecBuilder.fill(
                    tag.ftcPose.y * tag.ftcPose.y * 1.5,
                    tag.ftcPose.y * tag.ftcPose.y * 1.5,
                    tag.ftcPose.y * tag.ftcPose.y * 1.5));
            Telemetry.addDashboardData("AprilTags/detections/" + i + "/robotPose", robotPose);
          }
        }
      }
      Telemetry.addDashboardData("AprilTags/detections/" + i + "/id", tag.id);
    }
    Telemetry.addDashboardData("Drivetrain/Pinpoint pose", poseEstimator.getEstimatedPosition());
    Telemetry.addDSData("Detected Motif", motif == null ? "None" : motif.name());
    Telemetry.addDashboardData("ZeroPose", Pose3d.kZero);

    var goal = onRed ? redGoal : blueGoal;
    Telemetry.addDashboardData("Drivetrain/AutoAim/Goal", new Pose2d(goal, Rotation2d.kZero));
    var currentPose = poseEstimator.getEstimatedPosition();
    currentPoseToGoalDelta = goal.minus(currentPose.getTranslation());
    distanceToGoalMeters = currentPoseToGoalDelta.getNorm();
    Telemetry.addDashboardData("Drivetrain/AutoAim/distance", distanceToGoalMeters);
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
    return robotRelativeDrive(
        () -> {
          var xControl = xInput.getAsDouble();
          var yControl = yInput.getAsDouble();
          var translationalMagnitude = Math.hypot(xControl, yControl);
          xControl *= translationalMagnitude;
          yControl *= translationalMagnitude;
          var omegaControl = omegaInput.getAsDouble();
          omegaControl *= omegaControl * Math.signum(omegaControl);
          return ChassisSpeeds.fromFieldRelativeSpeeds(
              xControl * maxSpeedMetersPerSec,
              yControl * maxSpeedMetersPerSec,
              omegaControl * maxAngularSpeedRadPerSec,
              poseEstimator
                  .getEstimatedPosition()
                  .getRotation()
                  .plus(onRed ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg));
        });
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

  private Translation2d currentPoseToGoalDelta;
  private double distanceToGoalMeters;

  private Command aimAtGoal(Supplier<Translation2d> translationalControlSupplier) {
    return robotRelativeDrive(
        () -> {
          var translationalControl = translationalControlSupplier.get();
          var xControl = translationalControl.getX();
          var yControl = translationalControl.getY();

          var currentPose = poseEstimator.getEstimatedPosition();
          var currentPoseToGoalAngle = currentPoseToGoalDelta.getAngle();

          var feedback =
              yawPid.calculate(
                  currentPose.getRotation().getRadians(), currentPoseToGoalAngle.getRadians());
          var feedforward =
              (currentPoseToGoalAngle.getCos() * yControl
                      + currentPoseToGoalAngle.getSin() * xControl)
                  / distanceToGoalMeters;

          Telemetry.addDashboardData("Drivetrain/AutoAim/Feedback", feedback);
          Telemetry.addDashboardData("Drivetrain/AutoAim/Feedforward", feedforward);

          return ChassisSpeeds.fromFieldRelativeSpeeds(
              xControl,
              yControl,
              feedback + feedforward,
              poseEstimator.getEstimatedPosition().getRotation());
        });
  }

  public Command aimAtGoalTeleop(DoubleSupplier xInput, DoubleSupplier yInput) {
    return aimAtGoal(
        () -> {
          var xControl = xInput.getAsDouble();
          var yControl = yInput.getAsDouble();
          var translationalMagnitude = Math.hypot(xControl, yControl);
          xControl *= translationalMagnitude;
          yControl *= translationalMagnitude;
          if (onRed) {
            xControl *= -1;
            yControl *= -1;
          }
          return new Translation2d(xControl, yControl)
              .rotateBy(onRed ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
        });
  }

  public Command aimAtGoalAuto(Translation2d targetPose) {
    return aimAtGoal(
        () ->
            new Translation2d(
                xPid.calculate(poseEstimator.getEstimatedPosition().getX(), targetPose.getX()),
                yPid.calculate(poseEstimator.getEstimatedPosition().getY(), targetPose.getY())));
  }

  public double getDistanceToGoalMeters() {
    return distanceToGoalMeters;
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
