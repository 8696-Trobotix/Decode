// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.trobotix.hardware.ModeledMotor;
import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;

public class Flywheel extends SubsystemBase {
  private final ModeledMotor motor =
      new ModeledMotor(
          new Motor("Motor5"),
          new Encoder("Motor5", Encoder.CountsPerRevolution.GOBILDA_6000RPM),
          DCMotor.GOBILDA_5203_6000RPM(1),
          9,
          5);

    @Override
    public void periodic() {
        Telemetry.addDashboardData("Flywheel/Velocity RPM", motor.getEncoder().getVelocity() * 60);
    }

    public Command spinUp() {
    return run(
        () -> {
          double targetRPM = 5000;
          var feedforward = targetRPM * (12.0 / 6000);
          var feedback = (12.0 / 100) * (targetRPM / 60.0 - motor.getEncoder().getVelocity());
          motor.setVoltage(feedforward + feedback);
        });
  }
}
