// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.trobotix;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.HashMap;
import org.firstinspires.ftc.lib.trobotix.hardware.Encoder;
import org.firstinspires.ftc.lib.wpilib.command.CommandScheduler;
import org.firstinspires.ftc.lib.wpilib.command.button.CommandXboxController;
import org.firstinspires.ftc.lib.wpilib.command.button.Trigger;
import org.firstinspires.ftc.lib.wpilib.wpilibj.Timer;
import org.firstinspires.ftc.teamcode.Robot;

@Photon
public abstract class BaseOpMode extends LinearOpMode {
  private final String name;

  protected BaseOpMode() {
    var name = getClass().getName();
    this.name = name.substring(name.lastIndexOf('.') + 1);
  }

  @Override
  public final void runOpMode() {
    // Pre-user code initialization
    final var lynxModules = super.hardwareMap.getAll(LynxModule.class);
    for (var module : lynxModules) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    var voltageSensor = super.hardwareMap.getAll(LynxVoltageSensor.class).iterator().next();
    activeOpMode = name;
    BaseOpMode.hardwareMap = super.hardwareMap;
    Telemetry.setTelemetry(telemetry);

    Robot.init();
    if (!initializedOpModes.contains(activeOpMode)) {
      initialize();
      initializedOpModes.add(activeOpMode);
    }

    double dt = 1;
    while (!isStopRequested()) {
      telemetry.addData("Active Op Mode", name);
      telemetry.addData("Current time", Timer.getTimestamp());
      double startTime = Timer.getTimestamp();
      robotEnabled = opModeIsActive();
      for (var module : lynxModules) {
        module.clearBulkCache();
      }
      //noinspection ConstantValue
      telemetry.addData("Uncommited changes?", BuildConstants.DIRTY == 1 ? "YES" : "No");
      Encoder.recalculateVelocity(dt);
      busVoltage = voltageSensor.getVoltage();
      CommandScheduler.getInstance().run();
      dt = Timer.getTimestamp() - startTime;
      Telemetry.addDSData("Main loop time", dt);
      telemetry.update();
      Telemetry.sendDashboardTelemetry();
    }
    robotEnabled = false;
    CommandScheduler.getInstance().run();
    activeOpMode = null;
    BaseOpMode.hardwareMap = null;
    Telemetry.setTelemetry(null);
  }

  protected abstract void initialize();

  /**
   * The {@link Trigger} for when this specific OpMode is enabled. Can be safely called multiple
   * times, as it's cached.
   */
  protected final Trigger enabled() {
    return opModeEnableTriggers.computeIfAbsent(
        name, (name) -> new Trigger(() -> robotEnabled, name));
  }

  // TODO: See if we get a NPE when initializing, since gamepad1 & gamepad2 don't get set to a
  //  non-null value for a *while*
  /**
   * A {@link CommandXboxController} that wraps gamepad1 to use the Commands framework.
   *
   * <p>USERS SHOULD NOT USE GAMEPAD1 DIRECTLY, IT WILL NOT UPDATE. USE THIS INSTEAD.
   */
  protected final CommandXboxController primaryController =
      new CommandXboxController(() -> gamepad1);

  /**
   * A {@link CommandXboxController} that wraps gamepad2 to use the Commands framework.
   *
   * <p>USERS SHOULD NOT USE GAMEPAD2 DIRECTLY, IT WILL NOT UPDATE. USE THIS INSTEAD.
   */
  protected final CommandXboxController secondaryController =
      new CommandXboxController(() -> gamepad2);

  private static final ArrayList<String> initializedOpModes = new ArrayList<>();
  private static final HashMap<String, Trigger> opModeEnableTriggers = new HashMap<>();

  public static boolean robotEnabled = false;

  public static String activeOpMode = null;

  public static HardwareMap hardwareMap = null;

  public static double busVoltage = 12;
}
