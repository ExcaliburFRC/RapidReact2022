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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.robot.Constants.DrivetrainConstants;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
  private final CANSparkMax leftLeader =
      new CANSparkMax(DrivetrainConstants.LEFT_LEADER_ID, MotorType.kBrushless);

  @SuppressWarnings("FieldCanBeLocal")
  private final CANSparkMax leftFollower =
      new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);

  private final CANSparkMax rightLeader =
      new CANSparkMax(DrivetrainConstants.RIGHT_LEADER_ID, MotorType.kBrushless);

  @SuppressWarnings("FieldCanBeLocal")
  private final CANSparkMax rightFollower =
      new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();

  private final SparkMaxPIDController leftController = leftLeader.getPIDController();
  private final SparkMaxPIDController rightController = rightLeader.getPIDController();
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  private final PIDController rotationController = new PIDController(kP_ang, 0, 0);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();
  private final AHRS gyro = new AHRS();
  private final RamseteController ramseteController = new RamseteController();

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
        leftLeader.setSmartCurrentLimit(50),
        leftFollower.setSmartCurrentLimit(50),
        rightLeader.setSmartCurrentLimit(50),
        rightFollower.setSmartCurrentLimit(50),
        leftEncoder.setPositionConversionFactor(MOTOR_ROTATION_TO_METERS),
        rightEncoder.setPositionConversionFactor(MOTOR_ROTATION_TO_METERS),

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
    rotationController.enableContinuousInput(-180, 180);
  }

  public Command setMaxOutput(double output){
    return new InstantCommand(()-> drive.setMaxOutput(output));
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
    field.setRobotPose(
        odometry.update(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    SendableRegistry.remove(gyro);
    SendableRegistry.remove(drive);
    builder.addDoubleProperty(
        "gyro", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
    SmartDashboard.putData("Field", field);
    builder.addDoubleProperty("distance from hub", this::getDistanceFromHub, null);
    builder.addDoubleProperty("angle turn to hub", this::getAngleFromHub, null);
  }

  public double getDistanceFromHub() {
    Translation2d myTranslation2d = odometry.getPoseMeters().getTranslation();
    return myTranslation2d.getDistance(HUB_POS);
  }

  public double getAngleFromHub() {
    Pose2d myPos = odometry.getPoseMeters();
    double Dx = HUB_POS.getX() - myPos.getTranslation().getX();
    double Dy = HUB_POS.getY() - myPos.getTranslation().getY();
    return new Rotation2d(Dy, Dx).minus(myPos.getRotation()).getDegrees();
  }

  public void achieveVelocity(double left, double right) {
    leftController.setReference(
        left, ControlType.kVelocity, 0, feedforward.calculate(left), ArbFFUnits.kVoltage);
    rightController.setReference(
        right, ControlType.kVelocity, 0, feedforward.calculate(right), ArbFFUnits.kVoltage);
  }

  public Command rotateToHub() {
    return new PIDCommand(
            rotationController, this::getAngleFromHub, 0, rot -> drive.arcadeDrive(0, rot), this)
        .until(() -> rotationController.getPositionError() > 5);
  }

  public Command followTrajectoryCommand(Trajectory trajectory) {
    return resetOdometryCommand(trajectory.getInitialPose())
        .andThen(
            new RamseteCommand(
                trajectory,
                odometry::getPoseMeters,
                ramseteController,
                driveKinematics,
                this::achieveVelocity,
                this));
  }

  public Command followTrajectoryCommand(
        Pose2d start,
        List<Translation2d> interiorWaypoints,
        Pose2d end,
        TrajectoryConfig config){
    return followTrajectoryCommand(TrajectoryGenerator.generateTrajectory(
          start, interiorWaypoints, end, config));
  }

  public Command rotateToAngleCommand(double degrees) {
    return new PIDCommand(
            rotationController,
            () -> odometry.getPoseMeters().getRotation().getDegrees(),
            degrees,
            output -> drive.arcadeDrive(0, output),
            this)
        .until(new Trigger(() -> rotationController.getPositionError() < 5).debounce(0.1));
  }
}
