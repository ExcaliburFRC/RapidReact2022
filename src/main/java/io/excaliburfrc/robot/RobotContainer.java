package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.*;

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

  private final ShootBallsCommand ballShooter =
      new ShootBallsCommand(intake, shooter, LEDs.getInstance());

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
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    final int intakeAxis = 3;
    final int upperAxis = 5;
    final int intakePiston = 2;

    intake
        .manualCommand(
            () -> armJoystick.getRawAxis(intakeAxis),
            () -> armJoystick.getRawAxis(upperAxis),
            () -> armJoystick.getRawButton(intakePiston))
        .schedule();

    drive.arcadeDriveCommend(driveJoystick::getLeftY, driveJoystick::getRightX).schedule();
    //    shooter.manualCommand(() -> armJoystick.getRawAxis(4));
    shooter.manualCommand(() -> armJoystick.getRawButtonPressed(11)).schedule();
    climber
        .climberManualCommand(
            () -> {
              if (driveJoystick.getTriangleButton()) {
                return CLIMB_SPEED;
              } else if (driveJoystick.getCrossButton()) {
                return -CLIMB_SPEED;
              } else {
                return 0;
              }
            },
            () -> {
              if (driveJoystick.getRawButton(POV_UP)) {
                return CLIMB_SPEED;
              } else if (driveJoystick.getRawButton(POV_DOWN)) {
                return -CLIMB_SPEED;
              } else {
                return 0;
              }
            },
            driveJoystick::getCircleButton)
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
