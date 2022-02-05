package io.excaliburfrc.lib;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

/**
 * A command that runs another command repeatedly, restarting it when it ends, until this command is
 * interrupted. While this class does not extend {@link CommandGroupBase}, it is still considered a
 * CommandGroup, as it allows one to compose another command within it; the command instances that
 * are passed to it cannot be added to any other groups, or scheduled individually.
 *
 * <p>As a rule, CommandGroups require the union of the requirements of their component commands.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RepeatingCommand extends CommandBase {
  protected final Command m_command;

  /**
   * Creates a new RepeatingCommand. Will run another command repeatedly, restarting it whenever it
   * ends, until this command is interrupted.
   *
   * @param command the command to run repeatedly
   */
  public RepeatingCommand(Command command) {
    requireUngrouped(command);
    //    registerGroupedCommands(command); // FIXME package-private
    m_command = command;
    m_requirements.addAll(command.getRequirements());
  }

  @Override
  public void initialize() {
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
    if (m_command.isFinished()) {
      // restart command
      m_command.end(false);
      m_command.initialize();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_command.runsWhenDisabled();
  }
}
