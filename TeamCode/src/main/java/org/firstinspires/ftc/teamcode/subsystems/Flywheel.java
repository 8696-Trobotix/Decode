// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.trobotix.hardware.ModeledMotor;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.lib.wpilib.math.filter.Debouncer;
import org.firstinspires.ftc.lib.wpilib.math.filter.LinearFilter;
import org.firstinspires.ftc.lib.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;

public class Flywheel extends SubsystemBase {
  private final DoubleSupplier targetDistanceSupplier;

  private final InterpolatingDoubleTreeMap shotTable = new InterpolatingDoubleTreeMap();

  public Flywheel(DoubleSupplier targetDistanceSupplier) {
    this.targetDistanceSupplier = targetDistanceSupplier;

    shotTable.put(1.0, 3000.0);
    shotTable.put(1.25, 3100.0);
    shotTable.put(1.5, 3500.0);
    shotTable.put(1.75, 3750.0);
    shotTable.put(2.0, 3850.0);
    shotTable.put(2.25, 4020.0);
    shotTable.put(2.5, 4200.0);

    BaseOpMode.addResetHook(rpmFilter::reset);
  }

  private final ModeledMotor motor =
      new ModeledMotor(
          new Motor("Motor5").withInvert(true),
          new Encoder("Motor5", Encoder.CountsPerRevolution.GOBILDA_6000RPM).withInvert(true),
          DCMotor.GOBILDA_5203_6000RPM(1),
          8,
          5);

  private double currentRPM;

  @Override
  public void periodic() {
    currentRPM = rpmFilter.calculate(motor.getEncoder().getVelocity() * 60);
    Telemetry.addDashboardData("Flywheel/Position Rotations", motor.getEncoder().getPosition());
    Telemetry.addDashboardData("Flywheel/Velocity RPM", currentRPM);
    Telemetry.addDashboardData("Flywheel/Target RPM", targetRPM);
    Telemetry.addDashboardData("Flywheel/At target RPM", isAtTargetRPM());
  }

  private double targetRPM;

  public Command shootAtGoal() {
    return shoot(() -> shotTable.get(targetDistanceSupplier.getAsDouble()));
  }

  public Command sort() {
    return shoot(() -> 2210);
  }

  private final LinearFilter rpmFilter = LinearFilter.singlePoleIIR(.1, .015);

  private final PIDController flywheelPID = new PIDController(0.005, 0, 0.00);

  private Command shoot(DoubleSupplier targetRPMSupplier) {
    return run(() -> {
          targetRPM = targetRPMSupplier.getAsDouble();
          var feedforward = targetRPM * (5.78 / 2250);
          var feedback = flywheelPID.calculate(currentRPM, targetRPM);
          Telemetry.addDSData("Flywheel/Commanded Voltage", feedforward + feedback);
          motor.setVoltage(feedforward + feedback);
        })
        .finallyDo(() -> motor.setVoltage(0));
  }

  private final Debouncer debouncer = new Debouncer(0.25);

  public boolean isAtTargetRPM() {
    return debouncer.calculate(Math.abs(targetRPM - currentRPM) < 150);
  }
}
