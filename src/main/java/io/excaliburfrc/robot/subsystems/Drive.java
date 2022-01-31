package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.DrivetrainConstants.*;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.DrivetrainConstants;
import java.util.List;
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

  private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, 10);
  private final TrajectoryConfig config =
      new TrajectoryConfig(
              DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
              DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(driveKinematics)
          .addConstraint(autoVoltageConstraint);
  public final Trajectory trajectory =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

  private final DifferentialDriveOdometry odometry;
  private final AHRS ahrs = new AHRS();
  private final SparkMaxPIDController leftController = leftLeader.getPIDController();
  private final SparkMaxPIDController rightController = rightLeader.getPIDController();
  private Rotation2d offset = new Rotation2d(0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public Drive() {
    ValidateREVCAN(
        // reset factory settings
        leftLeader.restoreFactoryDefaults(),
        leftFollower.restoreFactoryDefaults(),
        rightLeader.restoreFactoryDefaults(),
        rightFollower.restoreFactoryDefaults(),
        // set the motors to coast mode -- we don't want to break them!
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake),
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake),
        rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake),
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake),
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

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    offset = pose.getRotation();
    odometry.resetPosition(pose, ahrs.getRotation2d());
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

  public Command arcadeDriveCommend(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return new RunCommand(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), this);
  }

  public void tankDrive(double left, double right) {
    leftLeader.set(left);
    rightLeader.set(right);
    drive.feed();
  }

  public RamseteCommand ramseteCommand() {
    return new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(),
        driveKinematics,
        this::achieveVelocity,
        this);
  }

  @Override
  public void periodic() {
    odometry.update(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
