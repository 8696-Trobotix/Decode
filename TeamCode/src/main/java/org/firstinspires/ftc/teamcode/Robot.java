// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

public class Robot {
  private static Robot instance = null;

  public static void init() {
    if (instance == null) {
      instance = new Robot();
    }
  }

  public static Robot getInstance() {
    init();
    return instance;
  }

  public final Drivetrain drivetrain;
  public final Flywheel flywheel;

  private Robot() {
    drivetrain = new Drivetrain();
    flywheel = new Flywheel();
  }
}
