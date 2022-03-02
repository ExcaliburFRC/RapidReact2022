package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final Drive drive = new Drive();
  private final LEDs leds = new LEDs();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public RobotContainer() {
    // Configure the button bindings
  }

  private static final int POV_UP = 0;
  private static final int POV_DOWN = 180;

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    var fenderShotCommand = shooter.accelerateFenderCommand();
    new POVButton(armJoystick, POV_UP).whenPressed(fenderShotCommand);
    new POVButton(armJoystick, POV_DOWN).whenPressed(fenderShotCommand);
  }

  void manualButton() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    intake.setDefaultCommand(
        intake.manualCommand(
            () -> armJoystick.getRawButton(2),
            () -> armJoystick.getRawButton(4),
            () -> armJoystick.getRawButton(5),
            () -> armJoystick.getRawButton(6),
            () -> armJoystick.getRawButtonPressed(8)));
    leds.setDefaultCommand(leds.setColorCommand(LEDs.LedMode.GOLD));

    new JoystickButton(armJoystick, 11).whenPressed(intake.intakeBallCommand());

    //    drive.arcadeDriveCommend(()-> -driveJoystick.getLeftY(),
    // driveJoystick::getRightX).schedule();
    drive.arcadeDriveCommend(() -> -driveJoystick.getLeftY(), driveJoystick::getRightX).schedule();
    new JoystickButton(armJoystick, 1).whenHeld(shooter.accelerateFenderCommand());
    new JoystickButton(armJoystick, 9).whenHeld(shooter.manualCommand());

    //    drive.arcadeDriveCommend(()-> -driveJoystick.getLeftY(),
    // driveJoystick::getRightX).schedule();
    drive.arcadeDriveCommend(() -> -driveJoystick.getLeftY(), driveJoystick::getRightX).schedule();
    new JoystickButton(armJoystick, 1).whenHeld(shooter.accelerateFenderCommand());
    new JoystickButton(armJoystick, 9).whenHeld(shooter.manualCommand());
    var CLIMB_SPEED = 0.4;
    climber
        .climberManualCommand(
            () -> {
              if (driveJoystick.getPOV() == POV_UP) {
                return CLIMB_SPEED;
              } else if (driveJoystick.getPOV() == POV_DOWN) {
                return -CLIMB_SPEED;
              } else {
                return 0;
              }
            },
            () -> {
              if (driveJoystick.getCrossButton()) {
                return -CLIMB_SPEED;
              } else if (driveJoystick.getTriangleButton()) {
                return CLIMB_SPEED;
              } else {
                return 0;
              }
            },
            driveJoystick::getOptionsButtonPressed,
            driveJoystick::getShareButtonPressed)
        .schedule();

    new Button(driveJoystick::getTouchpad)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));
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
