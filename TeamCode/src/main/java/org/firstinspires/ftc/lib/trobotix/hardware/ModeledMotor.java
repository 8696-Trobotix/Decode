// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.trobotix.Telemetry;
import org.firstinspires.ftc.lib.wpilib.math.system.plant.DCMotor;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class ModeledMotor {
  private final Motor motor;
  private final DCMotor motorModel;
  private final Encoder encoder;
  private final double statorCurrentLimitAmps, supplyCurrentLimitAmps;

  public ModeledMotor(
      Motor motor,
      Encoder encoder,
      DCMotor motorModel,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps) {
    this.motor = motor;
    this.motorModel = motorModel;
    this.encoder = encoder;
    this.statorCurrentLimitAmps = statorCurrentLimitAmps;
    this.supplyCurrentLimitAmps = supplyCurrentLimitAmps;
  }

  public void setVoltage(double volts) {
    double velRadPerSec = Units.rotationsToRadians(encoder.getVelocity());

    if (volts >= 0) {
      volts = Math.min(volts, getVoltageLimit(velRadPerSec));
    } else {
      volts = Math.max(volts, -getVoltageLimit(-velRadPerSec));
    }
    Telemetry.addData("TestMotor/Actual Voltage", volts);

    motor.setVoltage(volts);
  }

  private double getVoltageLimit(double velRadPerSec) {
    double statorLimit =
        Math.min(
            statorCurrentLimitAmps,
            (-(motorModel.stallCurrentAmps - motorModel.freeCurrentAmps)
                * (velRadPerSec / motorModel.freeSpeedRadPerSec)
                + Math.sqrt(
                Math.pow(
                    (motorModel.stallCurrentAmps - motorModel.freeCurrentAmps)
                        * (velRadPerSec / motorModel.freeSpeedRadPerSec),
                    2)
                    + 4
                    * (BaseOpMode.busVoltage / 12)
                    * motorModel.stallCurrentAmps
                    * supplyCurrentLimitAmps))
                / 2);
    Telemetry.addData(
        "TestMotor/Math vel (RPM)", Units.radiansPerSecondToRotationsPerMinute(velRadPerSec));
    Telemetry.addData("TestMotor/Stator Limit", statorLimit);
    return motorModel.getVoltage(motorModel.getTorque(statorLimit), velRadPerSec);
  }

  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
    encoder.setInverted(inverted);
  }

  public void setBrake(boolean brake) {
    motor.setInverted(brake);
  }

  public DcMotorEx getInternalMotor() {
    return motor.getInternalMotor();
  }

  public double getDutyCycle() {
    return motor.getDutyCycle();
  }

  public Encoder getEncoder() {
    return encoder;
  }
}
