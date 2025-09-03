// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import org.firstinspires.ftc.lib.trobotix.BaseOpMode;

public class Encoder {
  private final DcMotorEx internalMotor;
  private final double countsPerRevolution;

  private static final ArrayList<DoubleConsumer> velocityCacheResets = new ArrayList<>();

  public static void recalculateVelocity(double dt) {
    for (var cacheReset : velocityCacheResets) {
      cacheReset.accept(dt);
    }
  }

  public Encoder(String name, double encoderGearReduction) {
    internalMotor = (DcMotorEx) BaseOpMode.hardwareMap.dcMotor.get(name);
    this.countsPerRevolution = encoderGearReduction;
    velocityCacheResets.add(
        (dt) -> {
          if (dt < .1) {
            cachedVelocity = (getPosition() - lastPos) / dt;
          } else {
            cachedVelocity = internalMotor.getVelocity() / countsPerRevolution;
          }
          lastPos = getPosition();
        });
    lastPos = getPosition();
  }

  public double getPosition() {
    return internalMotor.getCurrentPosition() / countsPerRevolution;
  }

  private double lastPos;
  private double cachedVelocity = 0;

  public double getVelocity() {
    return cachedVelocity;
  }
}
