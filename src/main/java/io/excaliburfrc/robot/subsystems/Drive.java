package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.robot.Constants.DrivetrainConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.DrivetrainConstants;
import java.util.function.BooleanSupplier;
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

  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH_METERS);

  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro = new AHRS();
  private final SparkMaxPIDController leftController = leftLeader.getPIDController();
  private final SparkMaxPIDController rightController = rightLeader.getPIDController();
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

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
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DEFAULT),
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DEFAULT),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DEFAULT),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DEFAULT),
        // other status frames can be reduced to almost never
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DO_NOT_SEND),
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND),
        rightFollower.setPeriodicFramePeriod(
            PeriodicFrame.kStatus0, StatusFramePeriods.DO_NOT_SEND),
        rightFollower.setPeriodicFramePeriod(
            PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        rightFollower.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND));

    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    ValidateREVCAN(
        // setup following
        leftFollower.follow(leftLeader, false), rightFollower.follow(rightLeader, false));

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    drive.setSafetyEnabled(false);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void achieveVelocity(double left, double right) {
    leftController.setReference(
        left, ControlType.kVelocity, 0, feedforward.calculate(left), ArbFFUnits.kVoltage);
    rightController.setReference(
        right, ControlType.kVelocity, 0, feedforward.calculate(right), ArbFFUnits.kVoltage);
  }

  public Command arcadeDriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return arcadeDriveCommand(xSpeed, zRotation, () -> false);
  }

  public Command arcadeDriveCommand(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier slowMode) {
    return new RunCommand(
        () ->
            drive.arcadeDrive(
                xSpeed.getAsDouble() * (slowMode.getAsBoolean() ? 0.5 : 1),
                zRotation.getAsDouble()),
        this);
  }

  public Command curvatureDriveCommand(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier quickTurn) {
    return new RunCommand(
        () ->
            drive.curvatureDrive(
                xSpeed.getAsDouble(), zRotation.getAsDouble(), quickTurn.getAsBoolean()),
        this);
  }

  public Command followTrajectoryCommand(Trajectory trajectory) {
    return resetOdometryCommand(trajectory.getInitialPose()).andThen(ramseteCommand(trajectory));
  }

  public RamseteCommand ramseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(),
        driveKinematics,
        this::achieveVelocity,
        this);
  }

  public Command resetOdometryCommand(Pose2d pose) {
    return new InstantCommand(
        () -> {
          odometry.resetPosition(pose, gyro.getRotation2d());
          leftEncoder.setPosition(0);
          rightEncoder.setPosition(0);
        },
        this);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    SendableRegistry.remove(gyro);
    SendableRegistry.remove(drive);

    builder.addDoubleProperty(
        "heading", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
    builder.addDoubleProperty("x", () -> odometry.getPoseMeters().getX(), null);
    builder.addDoubleProperty("y", () -> odometry.getPoseMeters().getY(), null);
  }
}
