package io.excaliburfrc.robot;

import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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

    final int intakeButton = 6;
    final int spitButton = 3;
    final int transporterButton = 2;
    final int splitTransporterButton = 4;
    final int intakePiston = 10;

    intake
        .manualCommand(
            () -> armJoystick.getRawButton(intakeButton),
            () -> armJoystick.getRawButton(transporterButton),
            () -> armJoystick.getRawButton(intakePiston),
            () -> armJoystick.getRawButton(spitButton),
            () -> armJoystick.getRawButton(splitTransporterButton))
        .schedule();

//    drive.tankDriveCommand(driveJoystick::getLeftY, driveJoystick::getRightY).schedule();
//    drive.arcadeDriveCommend(armJoystick::getY, armJoystick::getX).schedule();
        shooter.manualCommand(() -> armJoystick.getRawAxis(4)).schedule();
//    new JoystickButton(armJoystick, 11).toggleWhenPressed(shooter.manualCommand(() -> 0.5));
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
              if (driveJoystick.getCircleButton()) {
                return CLIMB_SPEED;
              } else if (driveJoystick.getSquareButton()) {
                return -CLIMB_SPEED;
              } else {
                return 0;
              }
            },
            driveJoystick::getL1Button)
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
