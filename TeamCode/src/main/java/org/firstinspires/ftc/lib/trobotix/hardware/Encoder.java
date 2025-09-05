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

  private double velTMinus1 = 0;
  private double velTMinus2 = 0;

  public Encoder(String name, double encoderGearReduction) {
    internalMotor = (DcMotorEx) BaseOpMode.hardwareMap.dcMotor.get(name);
    this.countsPerRevolution = encoderGearReduction;
    velocityCacheResets.add(
        (dt) -> {
          var currentVel = (getPosition() - lastPos) / dt;
          cachedVelocity = (currentVel + velTMinus1 + velTMinus2) / 3;
          velTMinus2 = velTMinus1;
          velTMinus1 = currentVel;
          lastPos = getPosition();
        });
    lastPos = getPosition();
  }

  private boolean inverted = false;

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public double getPosition() {
    return (inverted ? -internalMotor.getCurrentPosition() : internalMotor.getCurrentPosition())
        / countsPerRevolution;
  }

  private double lastPos;
  private double cachedVelocity = 0;

  public double getVelocity() {
    return cachedVelocity;
  }

  public static final class CountsPerRevolution {
    public static final double GOBILDA_6000RPM = 28;
    public static final double GOBILDA_1620RPM = GOBILDA_6000RPM * (1 + (46.0 / 17.0));
    public static final double GOBILDA_1159RPM = GOBILDA_6000RPM * (1 + (46.0 / 11.0));
    public static final double GOBILDA_435RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)));
    public static final double GOBILDA_312RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)));
    public static final double GOBILDA_223RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
    public static final double GOBILDA_117RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)));
    public static final double GOBILDA_84RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)));
    public static final double GOBILDA_60RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
    public static final double GOBILDA_43RPM =
        GOBILDA_6000RPM * ((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
    public static final double GOBILDA_30RPM =
        GOBILDA_6000RPM
            * ((1 + (46.0 / 17.0))
                * (1 + (46.0 / 17.0))
                * (1 + (46.0 / 17.0))
                * (1 + (46.0 / 17.0)));
  }
}
