package io.excaliburfrc.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that runs a given runnable continuously in it's execute method until interrupted,
 * and another runnable when it ends.
 * Useful for running a motor and making sure it stops when the command ends even if it doesn't get any other values.
 * Has no end condition as-is; either subclass it or use {@link Command#withTimeout(double)} or
 * {@link Command#until(java.util.function.BooleanSupplier)} to give it one.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RunEndCommand extends CommandBase {
	protected final Runnable m_onExacute;
	protected final Runnable m_onEnd;

	/**
	 * Creates a new RunEndCommand. Will run the given runnable continuously until interrupted and then run the
	 * second runnable once
	 *
	 * @param toRun the Runnable to run continuously
	 * @param onEnd the Runnable to run on command end
	 * @param requirements the subsystems required by this command
	 */
	public RunEndCommand(Runnable toRun, Runnable onEnd, Subsystem... requirements) {
		m_onExacute = requireNonNullParam(toRun, "toRun", "RunEndCommand");
		m_onEnd = requireNonNullParam(onEnd, "onEnd", "RunEndCommand");

		addRequirements(requirements);
	}

	@Override
	public void execute() {
		m_onExacute.run();
	}

	@Override
	public void end(boolean interrupted) {
		m_onEnd.run();
	}
}
