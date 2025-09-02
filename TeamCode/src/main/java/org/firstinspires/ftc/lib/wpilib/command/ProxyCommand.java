// Copyright (c) 2024-2025 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.command;

import static org.firstinspires.ftc.lib.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.function.Supplier;

/**
 * Schedules a given command when this command is initialized and ends when it ends, but does not
 * directly run it. Use this for including a command in a composition without adding its
 * requirements, <strong>but only if you know what you are doing. If you are unsure, see <a
 * href="https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html#scheduling-other-commands">the
 * WPILib docs</a> for a complete explanation of proxy semantics.</strong> Do not proxy a command
 * from a subsystem already required by the composition, or else the composition will cancel itself
 * when the proxy is reached. If this command is interrupted, it will cancel the command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class ProxyCommand extends Command {
  private final Supplier<Command> m_supplier;
  private Command m_command;

  /**
   * Creates a new ProxyCommand that schedules the given command when initialized, and ends when it
   * is no longer scheduled.
   *
   * @param command the command to run by proxy
   */
  @SuppressWarnings("this-escape")
  public ProxyCommand(Command command) {
    Command nullCheckedCommand = requireNonNullParam(command, "command", "ProxyCommand");
    m_supplier = () -> nullCheckedCommand;
    setName("Proxy(" + nullCheckedCommand.getName() + ")");
  }

  @Override
  public void initialize() {
    m_command = m_supplier.get();
    m_command.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_command.cancel();
    }
    m_command = null;
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // because we're between `initialize` and `end`, `m_command` is necessarily not null
    // but if called otherwise and m_command is null,
    // it's UB, so we can do whatever we want -- like return true.
    return m_command == null || !m_command.isScheduled();
  }

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   *
   * @return true. Otherwise, this proxy would cancel commands that do run when disabled.
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
