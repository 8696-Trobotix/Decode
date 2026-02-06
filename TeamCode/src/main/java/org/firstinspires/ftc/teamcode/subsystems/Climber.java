package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.lib.trobotix.hardware.Motor;
import org.firstinspires.ftc.lib.wpilib.command.Command;
import org.firstinspires.ftc.lib.wpilib.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final Motor motor = new Motor("Motor6");

  public Climber() {
    motor.setBrake(true);
  }

  public Command climb() {
    return run(() -> motor.set(1)).finallyDo(() -> motor.set(0));
  }

  public Command unclimb() {
    return run(() -> motor.set(-1)).finallyDo(() -> motor.set(0));
  }
}
