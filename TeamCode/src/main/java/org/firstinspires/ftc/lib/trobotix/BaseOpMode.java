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
import org.firstinspires.ftc.teamcode.BuildConstants;

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
    Telemetry.initTelemetry();

    initialize();

    for (var hook : resetHooks) {
      hook.run();
    }

    double dt;
    double lastVelUpdateTime = Timer.getTimestampSeconds();
    while (!isStopRequested()) {
      double startTime = Timer.getTimestampSeconds();
      telemetry.addData("Active Op Mode", name);
      telemetry.addData("Current time", startTime);
      telemetry.addData("Git SHA", BuildConstants.GIT_SHA);
      //noinspection ConstantValue
      telemetry.addData("Uncommited changes?", BuildConstants.DIRTY == 1 ? "YES" : "No");
      for (var module : lynxModules) {
        module.getBulkData();
      }
      robotEnabled = isStarted();
      double lastVelUpdateDt = startTime - lastVelUpdateTime;
      if (lastVelUpdateDt > (1.0 / 50)) {
        Encoder.recalculateVelocity(lastVelUpdateDt);
        lastVelUpdateTime = startTime;
      }
      busVoltage = voltageSensor.getVoltage();
      CommandScheduler.getInstance().run();
      dt = Timer.getTimestampSeconds() - startTime;
      Telemetry.logRobotStats(dt, busVoltage);
      telemetry.update();
      Telemetry.sendDashboardTelemetry();
    }
    robotEnabled = false;
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().clearComposedCommands();
    activeOpMode = null;
    BaseOpMode.hardwareMap = null;
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

  private static final ArrayList<Runnable> resetHooks = new ArrayList<>();

  public static void addResetHook(Runnable hook) {
    resetHooks.add(hook);
  }

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

  private static final HashMap<String, Trigger> opModeEnableTriggers = new HashMap<>();

  public static boolean robotEnabled = false;

  public static String activeOpMode = null;

  public static HardwareMap hardwareMap = null;

  public static double busVoltage = 12;
}
