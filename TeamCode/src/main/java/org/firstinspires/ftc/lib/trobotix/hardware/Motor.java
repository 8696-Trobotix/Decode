// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;
import org.firstinspires.ftc.lib.wpilib.math.MathUtil;

public class Motor {
  private final DcMotorEx internalMotor;

  public Motor(String name) {
    internalMotor = (DcMotorEx) BaseOpMode.hardwareMap.dcMotor.get(name);
  }

  public void setBrake(boolean brake) {
    internalMotor.setZeroPowerBehavior(
        brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
  }

  public boolean inverted = false;

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  private double lastDutyCycle = 0.0;
  private double tolerance = .005;

  public void set(double dutyCycle) {
    dutyCycle = MathUtil.clamp(dutyCycle, -1, 1);
    if (dutyCycle == 0 || !MathUtil.isNear(lastDutyCycle, dutyCycle, tolerance)) {
      internalMotor.setPower(inverted ? -dutyCycle : dutyCycle);
      lastDutyCycle = dutyCycle;
    }
  }

  public void setVoltage(double volts) {
    set(volts / BaseOpMode.busVoltage);
  }

  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
  }

  public DcMotorEx getInternalMotor() {
    return internalMotor;
  }

  public double getDutyCycle() {
    return lastDutyCycle;
  }
}
