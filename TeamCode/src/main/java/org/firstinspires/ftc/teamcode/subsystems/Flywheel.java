// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.trobotix.hardware.ModeledMotor;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class Flywheel extends SubsystemBase {
  private final DoubleSupplier targetDistanceSupplier;

  private final InterpolatingDoubleTreeMap shotTable = new InterpolatingDoubleTreeMap();

  public Flywheel(DoubleSupplier targetDistanceSupplier) {
    this.targetDistanceSupplier = targetDistanceSupplier;

    shotTable.put(2.0, 3650.0);
    shotTable.put(1.75, 3300.0);
    shotTable.put(1.5, 3100.0);
    shotTable.put(1.33, 2950.0);
    shotTable.put(Units.feetToMeters(0.5) * Math.sqrt(2), 2750.0);
  }

  private final ModeledMotor motor =
      new ModeledMotor(
          new Motor("Motor5"),
          new Encoder("Motor5", Encoder.CountsPerRevolution.GOBILDA_6000RPM),
          DCMotor.GOBILDA_5203_6000RPM(1),
          8,
          5);

  @Override
  public void periodic() {
    Telemetry.addDashboardData("Flywheel/Position Rotations", motor.getEncoder().getPosition());
    Telemetry.addDashboardData("Flywheel/Velocity RPM", motor.getEncoder().getVelocity() * 60);
    Telemetry.addDashboardData("Flywheel/Target RPM", targetRPM);
  }

  private double targetRPM;

  public Command spinUp() {
    return run(() -> {
          targetRPM = shotTable.get(targetDistanceSupplier.getAsDouble());
          var feedforward = targetRPM * (10.0 / 4000);
          var feedback = (.7) * (targetRPM / 60.0 - motor.getEncoder().getVelocity());
          Telemetry.addDSData("Flywheel/Commanded Voltage", feedforward + feedback);
          motor.setVoltage(feedforward + feedback);
        })
        .finallyDo(() -> motor.setVoltage(0));
  }

  public boolean isAtTargetRPM() {
    return Math.abs(motor.getEncoder().getVelocity() * 60 - targetRPM) < 50;
  }
}
