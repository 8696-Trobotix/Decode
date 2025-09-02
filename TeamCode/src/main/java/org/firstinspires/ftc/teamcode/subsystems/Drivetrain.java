// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.Commands;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;
import org.firstinspires.ftc.lib.wpilib.math.kinematics.ChassisSpeeds;

public class Drivetrain extends SubsystemBase {
  public Command drive(ChassisSpeeds speeds) {
    return Commands.idle(this); // TODO: Implement
  }
}
