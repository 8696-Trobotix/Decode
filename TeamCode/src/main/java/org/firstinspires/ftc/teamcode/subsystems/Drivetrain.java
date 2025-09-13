// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.Supplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.estimator.PinpointPoseEstimator;
import org.firstinspires.ftc.lib.trobotix.hardware.AprilTagDetectorIO;
import org.firstinspires.ftc.lib.trobotix.hardware.AprilTagDetectorImpl;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.trobotix.hardware.ModeledMotor;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.trobotix.hardware.Pinpoint;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.psilynx.psikit.core.Logger;

public class Drivetrain extends SubsystemBase {
  private final ModeledMotor frontLeft, frontRight, backLeft, backRight;

  private final PinpointPoseEstimator poseEstimator;
  private final AprilTagDetectorIO detectorIO;

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
    frontLeft.setBrake(true);
    frontRight.setBrake(true);
    backLeft.setBrake(true);
    backRight.setBrake(true);

    poseEstimator =
        new PinpointPoseEstimator(
            new Pinpoint("Pinpoint", 0, 0, false, false),
            VecBuilder.fill(.1, .1, .1),
            VecBuilder.fill(.9, .9, .9));
    if (!Logger.isReplay()) {
      detectorIO =
          new AprilTagDetectorImpl(
              new Transform3d(),
              900, // TODO: Actually obtain an Arducam OV9281 and calibrate it
              639.5,
              900,
              399.5);
    } else {
      detectorIO = new AprilTagDetectorIO() {};
    }

    BaseOpMode.addResetHook(() -> {});
  }

  private enum Motif {
    GPP,
    PGP,
    PPG
  }

  private Motif motif = null;

  public final AprilTagDetectorIO.AprilTagDetectorIOInputs inputs =
      new AprilTagDetectorIO.AprilTagDetectorIOInputs();

  @Override
  public void periodic() {
    poseEstimator.update();
    detectorIO.updateInputs(inputs);
    Logger.processInputs("AprilTags", inputs);
    for (int i = 0; i < inputs.tagIDs.length; i++) {
      switch (inputs.tagIDs[i]) {
        case 21 -> motif = Motif.GPP;
        case 22 -> motif = Motif.PGP;
        case 23 -> motif = Motif.PPG;
        default ->
            poseEstimator.addVisionMeasurement(inputs.poses[i].toPose2d(), inputs.timestamps[i]);
      }
    }
    Logger.recordOutput("Drive/Estimated Pose", poseEstimator.getEstimatedPosition());
  }

  private static final double wheelbaseLengthMeters = Units.inchesToMeters(10);
  private static final double wheelbaseWidthMeters = Units.inchesToMeters(10);
  private final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          new Translation2d(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(wheelbaseLengthMeters / 2, -wheelbaseWidthMeters / 2),
          new Translation2d(-wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2),
          new Translation2d(-wheelbaseLengthMeters / 2, -wheelbaseWidthMeters / 2));

  public static final double maxSpeedMetersPerSec =
      DCMotor.GOBILDA_5203_435RPM(1).freeSpeedRadPerSec * (52.0 / 1000);
  public static final double maxAngularSpeedRadPerSec =
      maxSpeedMetersPerSec / Math.hypot(wheelbaseLengthMeters / 2, wheelbaseWidthMeters / 2);
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
