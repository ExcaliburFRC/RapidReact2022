package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.commands.NoRamsete;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.subsystems.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;

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

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public RobotContainer() {
    // Configure the button bindings
  }

  private static final int POV_UP = 0;
  private static final int POV_DOWN = 180;

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(() -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    leds.setDefaultCommand(leds.setColorCommand(LedMode.GOLD));

    var cancelButton = new POVButton(armJoystick, POV_UP);

    var shootBalls = new BlindShootBallsCommand(intake, shooter, leds).beforeStarting(intake.setPistonCommand(Value.kReverse));
    new JoystickButton(armJoystick, 1).whileActiveOnce(shootBalls);
//    cancelButton.cancelWhenPressed(shootBalls);

    var intakeAuto = intake.intakeBallCommand();
    new JoystickButton(armJoystick, 2).whenPressed(intakeAuto);
    cancelButton.cancelWhenPressed(intakeAuto);

    new JoystickButton(armJoystick, 3).whileHeld(intake.ejectCommand());

    new JoystickButton(armJoystick, 11).whenPressed(climber.climbSeries(new JoystickButton(armJoystick, 12)));

    var CLIMB_SPEED = 0.4;
    climber.disableSoftLimits().schedule();
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
                    driveJoystick::getL2Button,
                    driveJoystick::getR2Button).schedule();

    new Button(driveJoystick::getShareButton)
            .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    DriverStation.reportWarning("Buttons!", false);
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

    new JoystickButton(armJoystick, 11).whenPressed(intake.intakeBallCommand());

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(() -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));
    new JoystickButton(armJoystick, 1).whenHeld(shooter.accelerateFenderCommand());
    new JoystickButton(armJoystick, 9).whenHeld(shooter.manualCommand());

    new JoystickButton(armJoystick, 12).whenHeld(new ShootBallsCommand(intake, shooter, leds));

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

    new Button(driveJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getCircleButton).whileActiveOnce(climber.openFullyCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new NoRamsete(drive, intake, shooter, leds);
  }
}
