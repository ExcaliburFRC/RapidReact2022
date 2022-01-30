package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
  private final CANSparkMax leftLeader =
      new CANSparkMax(DrivetrainConstants.LEFT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(DrivetrainConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  private final AnalogPotentiometer rightLineSensor =
      new AnalogPotentiometer(DrivetrainConstants.RIGHT_LINE_SENSOR_CHANNEL, 100);
  private final AnalogPotentiometer leftLineSensor =
      new AnalogPotentiometer(DrivetrainConstants.LEFT_LINE_SENSOR_CHANNEL, 100);
  private final Trigger rightTrigger =
      new Trigger(() -> rightLineSensor.get() < DrivetrainConstants.BLACK_THRESHOLD);
  private final Trigger leftTrigger =
      new Trigger(() -> leftLineSensor.get() < DrivetrainConstants.BLACK_THRESHOLD);

  public Drive() {
    ValidateREVCAN(
        // reset factory settings
        leftLeader.restoreFactoryDefaults(),
        leftFollower.restoreFactoryDefaults(),
        rightLeader.restoreFactoryDefaults(),
        rightFollower.restoreFactoryDefaults(),
        // set the motors to coast mode -- we don't want to break them!
        leftLeader.setIdleMode(IdleMode.kBrake),
        leftFollower.setIdleMode(IdleMode.kBrake),
        rightLeader.setIdleMode(IdleMode.kBrake),
        rightFollower.setIdleMode(IdleMode.kBrake),
        // have the leader send its applied output as frequently as possible,
        // to speed up follower response
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, minimal_FRAME_PERIOD),
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, minimal_FRAME_PERIOD),
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, minimal_FRAME_PERIOD),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, minimal_FRAME_PERIOD),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        // other status frames can be reduced to almost never
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        // setup following
        leftFollower.follow(leftLeader),
        rightFollower.follow(rightLeader));
  }

  public Command arcadeDriveCommend(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return new RunCommand(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), this);
  }

  private Command moveToLineCommand() {
    return arcadeDriveCommend(() -> 0.2, () -> 0)
        .withInterrupt(leftTrigger.or(rightTrigger))
        .andThen(
            new SelectCommand(
                () -> {
                  if (rightTrigger.get())
                    return alignCommand(rightTrigger, leftTrigger, rightLeader, leftLeader);
                  if (leftTrigger.get())
                    return alignCommand(leftTrigger, rightTrigger, leftLeader, rightLeader);
                  return new PrintCommand(
                      "ASSERTION FAILED!!!!!\nCall the programmers, something is borked");
                }));
  }

  private Command alignCommand(
      Trigger firstSensor,
      Trigger secondSensor,
      MotorController firstMotor,
      MotorController secondMotor) {
    return getSideReachCommand(secondMotor, secondSensor, 0.2)
        .andThen(getSideReachCommand(firstMotor, firstSensor, -0.2));
  }

  private FunctionalCommand getSideReachCommand(
      MotorController firstMotor, Trigger firstSensor, double speed) {
    return new FunctionalCommand(
        () -> {}, () -> firstMotor.set(speed), __ -> firstMotor.set(0), firstSensor);
  }
}
