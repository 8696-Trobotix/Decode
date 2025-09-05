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
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TestMotor extends SubsystemBase {
  private final ModeledMotor motor =
      new ModeledMotor(
          new Motor("TestMotor"),
          new Encoder("TestMotor", Encoder.CountsPerRevolution.GOBILDA_6000RPM),
          DCMotor.GOBILDA_5203_6000RPM(1),
          6,
          4);

  public TestMotor() {
    motor.setBrake(true);
  }

  @Override
  public void periodic() {
    var statorCurrent = motor.getInternalMotor().getCurrent(CurrentUnit.AMPS);
    Telemetry.addData("TestMotor/Stator Current", statorCurrent);
    Telemetry.addData("TestMotor/Supply Current", statorCurrent * Math.abs(motor.getDutyCycle()));
    Telemetry.addData("TestMotor/Position (Rotations)", motor.getEncoder().getPosition());
    Telemetry.addData("TestMotor/Velocity (RPM)", motor.getEncoder().getVelocity() * 60);
  }

  public Command move(DoubleSupplier input) {
    return run(
        () -> {
          var voltage = input.getAsDouble() * 12;
          Telemetry.addData("TestMotor/Commanded Voltage", voltage);
          motor.setVoltage(voltage);
          Telemetry.addData(
              "TestMotor/Actual Voltage", motor.getDutyCycle() * BaseOpMode.busVoltage);
        });
  }
}
