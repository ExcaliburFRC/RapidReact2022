package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.subsystems.Climber;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.Intake;
import io.excaliburfrc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PS4Controller driveJoystick = new PS4Controller(0);
  private final Joystick armJoystick = new Joystick(1);
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();

  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  private final Drive drive = new Drive();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private static final int POV_UP = 0;
  private static final int POV_DOWN = 180;

  private void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    var fenderShotCommand = shooter.accelerateFenderCommand();
    new POVButton(armJoystick, POV_UP).whenPressed(fenderShotCommand);
    new POVButton(armJoystick, POV_DOWN).whenPressed(fenderShotCommand);
  }

  void manualButton() {
    final int anglePiston = 4;

    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    final int intakeAxis = 1;
    final int upperAxis = 2;
    final int intakeButton = 3;

    intake
        .manualCommand(
            () -> armJoystick.getRawAxis(intakeAxis),
            () -> armJoystick.getRawAxis(upperAxis),
            () -> armJoystick.getRawButton(intakeButton))
        .schedule();

    drive.arcadeDriveCommend(driveJoystick::getLeftY, driveJoystick::getRightX).schedule();
    shooter.manualCommand(() -> 0.5 * armJoystick.getY()).schedule();
    climber
        .climberManualCommand(armJoystick::getY, () -> armJoystick.getRawButton(anglePiston))
        .schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
