// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

public class Robot {
  private Robot() {}

  public static final Drivetrain drivetrain;
  public static final Flywheel flywheel;
  public static final Feeder feeder;
  public static final Climber climber = new Climber();

  static {
    drivetrain = new Drivetrain();
    flywheel = new Flywheel(drivetrain::getDistanceToGoalMeters);
    feeder = new Feeder(flywheel::isAtTargetRPM);
  }
}
