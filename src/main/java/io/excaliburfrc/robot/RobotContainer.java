package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.auto.twoBalls.fiveFour;
import io.excaliburfrc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PS4Controller driveJoystick = new PS4Controller(0);
  //  private final PS4Controller armJoystick = new PS4Controller(1);
  // The robot's subsystems and commands are defined here...
  private final Superstructure superstructure = new Superstructure();
  private final Climber climber = new Climber();
  private final Drive drive = new Drive();
  private final LEDs leds = new LEDs();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final SendableChooser<Command> initialBallCounter = new SendableChooser<>();

  public RobotContainer() {
    //    chooser.addOption(
    //        "4Ball", new FourBallAuto(drive, superstructure, leds));
    SmartDashboard.putData("Autos", chooser);

    initialBallCounter.addOption(
        "1", new InstantCommand(() -> superstructure.intake.resetBallCounter(1)));
    initialBallCounter.addOption(
        "2", new InstantCommand(() -> superstructure.intake.resetBallCounter(2)));

    SmartDashboard.putData("initial balls", initialBallCounter);
  }

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new Button(driveJoystick::getR2Button)
        .toggleWhenPressed(superstructure.shootBallsCommand(leds));
    //    new Button(driveJoystick::getR2Button)
    //          .toggleWhenPressed(superstructure.shooter.manualCommand(()-> 0.5));

    //    new Button(driveJoystick::getCircleButton)
    //        .toggleWhenPressed(
    //            drive.rotateToHub().deadlineWith(leds.setColorCommand(LEDs.LedMode.YELLOW)));

    // when intake is required
    new Button(() -> CommandScheduler.getInstance().requiring(superstructure.intake) != null)
        .whenReleased(superstructure.intake.closePiston());

    new Button(driveJoystick::getL2Button).toggleWhenPressed(superstructure.intakeBallCommand());

    new Button(driveJoystick::getL1Button).toggleWhenPressed(superstructure.ejectBallCommand());

    climber
        .climberManualCommand(
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270,
            driveJoystick::getR1Button)
        .schedule();

    new Button(driveJoystick::getSquareButton)
        .toggleWhenPressed(
            new ConditionalCommand(
                superstructure.intake.closePiston(),
                superstructure.intake.openPiston(),
                superstructure.intake::isOpen));

    //    new Button(driveJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(driveJoystick, 0).whenPressed(superstructure.shooter.incrementTarget(1));
    new POVButton(driveJoystick, 180).whenPressed(superstructure.shooter.incrementTarget(-1));

    new Button(driveJoystick::getOptionsButton)
        .toggleWhenPressed(superstructure.intake.allowCommand());

    DriverStation.reportWarning("Buttons!", false);
  }

  void manualButton() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));

    climber
        .climberManualCommand(
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270,
            driveJoystick::getR1Button)
        .schedule();

    new Button(driveJoystick::getR1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    DriverStation.reportWarning("Manual!", false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new fiveFour(drive, leds, superstructure);
  }
}
