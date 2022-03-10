package io.excaliburfrc.robot;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import io.excaliburfrc.robot.commands.BlindShootBallsCommand;
import io.excaliburfrc.robot.commands.NoRamseteBottomFender;
import io.excaliburfrc.robot.commands.NoRamseteTopFender;
import io.excaliburfrc.robot.commands.ShootBallsCommand;
import io.excaliburfrc.robot.commands.auto.out.OutFromBottom;
import io.excaliburfrc.robot.commands.auto.out.OutFromTop;
import io.excaliburfrc.robot.commands.auto.shoot1.*;
import io.excaliburfrc.robot.commands.auto.shoot2.Cargo4BottomShoot2;
import io.excaliburfrc.robot.commands.auto.shoot2.Cargo5BottomShoot2;
import io.excaliburfrc.robot.commands.auto.shoot2.Cargo6TopShoot2;
import io.excaliburfrc.robot.commands.auto.shoot2.PushTheirBallAuto;
import io.excaliburfrc.robot.commands.auto.shoot3.BottomShoot1Cargoes4And5BottomShoot2;
import io.excaliburfrc.robot.commands.auto.shoot3.BottomShoot1Cargoes5AndTerminalBottomShoot2;
import io.excaliburfrc.robot.commands.auto.shoot4.FourBallsAuto;
import io.excaliburfrc.robot.subsystems.*;
import io.excaliburfrc.robot.subsystems.LEDs.LedMode;
import java.lang.reflect.InvocationTargetException;
import java.util.List;

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

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    chooser.addOption("Top", new NoRamseteTopFender(drive, intake, shooter, leds));
    chooser.addOption("Bottom", new NoRamseteBottomFender(drive, intake, shooter, leds));
    SmartDashboard.putData("Autos", chooser);
    setupAutos();
  }

  void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    drive.setDefaultCommand(
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));
//  drive.setDefaultCommand(
//      drive.tankDriveCommand(
//            ()-> driveJoystick.getLeftY(), ()-> driveJoystick.getRightY()));

    leds.setDefaultCommand(leds.setColorCommand(leds.getAlliance()));

    new JoystickButton(armJoystick, 1)
        .whileActiveOnce(new BlindShootBallsCommand(intake, shooter, leds));

    new JoystickButton(armJoystick, 2)
        .whenReleased(intake.setPistonCommand(Value.kReverse))
        .whileActiveContinuous(intake.intakeBallCommand());

    new JoystickButton(armJoystick, 4).whileHeld(intake.ejectCommand());

    //    var stepButton = new Button(() -> armJoystick.getRawButton(3));
    //    new POVButton(driveJoystick, POV_UP)
    //        .whenPressed(
    //            climber.climbSeries(
    //                stepButton, stepButton, stepButton, stepButton, stepButton, stepButton));

    climber
        .climberManualCommand(
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270)
        .schedule();

    new Button(driveJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new POVButton(armJoystick, 0).whenPressed(shooter.incrementTarget(1));
    new POVButton(armJoystick, 180).whenPressed(shooter.incrementTarget(-1));

    new Button(driveJoystick::getOptionsButton).toggleWhenPressed(intake.allowCommand());

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
        drive.arcadeDriveCommand(
            () -> -driveJoystick.getLeftY(), driveJoystick::getRightX, driveJoystick::getR1Button));
    new JoystickButton(armJoystick, 1).whenHeld(shooter.accelerateFenderCommand());
    new JoystickButton(armJoystick, 9).whenHeld(shooter.manualCommand());

    new JoystickButton(armJoystick, 12).whenHeld(new ShootBallsCommand(intake, shooter, leds));

    climber
        .climberTuneCommand(
            () -> driveJoystick.getPOV() == 0,
            () -> driveJoystick.getPOV() == 180,
            driveJoystick::getTriangleButton,
            driveJoystick::getCrossButton,
            () -> driveJoystick.getPOV() == 90,
            () -> driveJoystick.getPOV() == 270)
        .schedule();

    new Button(driveJoystick::getShareButton)
        .toggleWhenPressed(new StartEndCommand(compressor::enableDigital, compressor::disable));

    new Button(driveJoystick::getL1Button).whileActiveOnce(climber.disableSoftLimits());

    new Button(driveJoystick::getCircleButton).whileActiveOnce(climber.openFullyCommand());

    DriverStation.reportWarning("Manual!", false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
        drive.resetOdometryCommand(new Pose2d(0, 0, fromDegrees(0))),
        drive.followTrajectoryCommand(
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, fromDegrees(0)),
                      new Pose2d(2, 0, fromDegrees(0))),
                new TrajectoryConfig(3, 3)))
              .alongWith(intake.intakeBallCommand()),
        drive.arcadeDriveCommand(() -> 0, () -> 0));
    //    return chooser.getSelected();
  }

  void setupAutos() {
    Class<? extends Command>[] classes =
        new Class[] {
          BottomShoot1AndOut.class,
          BottomShoot1Cargoes4And5BottomShoot2.class,
          BottomShoot1CollectCargo4.class,
          BottomShoot1CollectCargo4and5.class,
          BottomShoot1CollectCargo5.class,
          Cargo4BottomShoot2.class,
          Cargo5BottomShoot2.class,
          Cargo6TopShoot2.class,
          FourBallsAuto.class,
          OutFromBottom.class,
          OutFromTop.class,
          PushTheirBallAuto.class,
          BottomShoot1Cargoes5AndTerminalBottomShoot2.class,
          TopShoot1AndOut.class,
          TopShoot1CollectCargo6.class
        };
    for (Class<? extends Command> klass : classes) {
      try {
        chooser.addOption(
            klass.getSimpleName(),
            klass
                .getConstructor(Drive.class, Shooter.class, Intake.class, LEDs.class)
                .newInstance(drive, shooter, intake, leds));
      } catch (NoSuchMethodException
          | InstantiationException
          | IllegalAccessException
          | InvocationTargetException e) {
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
      }
    }
    SmartDashboard.putData(chooser);
  }
}
