package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import static java.lang.Math.abs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final CANSparkMax leftMotor =
      new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkMaxPIDController leftController = leftMotor.getPIDController();
  private final SparkMaxPIDController rightController = rightMotor.getPIDController();

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(0, MG * Math.cos(ANGLE), kV, kA);
  private double target;

  private double velocitySetpoint;

  public Climber() {
    ValidateREVCAN(
        // reset factory settings
        leftMotor.restoreFactoryDefaults(),
        rightMotor.restoreFactoryDefaults(),
        // set the motors to brake mode
        leftMotor.setIdleMode(IdleMode.kBrake),
        rightMotor.setIdleMode(IdleMode.kBrake),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        // set up PID parameters
        leftController.setFeedbackDevice(leftEncoder),
        leftController.setP(kP),
        leftController.setI(kI),
        leftController.setD(kD),
        rightController.setFeedbackDevice(rightEncoder),
        rightController.setP(kP),
        rightController.setI(kI),
        rightController.setD(kD));
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
    anglerPiston.close();
  }

  public enum MotorMode {
    OFF(0),
    UP(0.6),
    DOWN(-0.4);

    final double dutyCycle;

    MotorMode(double v) {
      dutyCycle = v;
    }
  }

  public void activateMotors(MotorMode m) {
    leftMotor.set(m.dutyCycle);
    rightMotor.set(m.dutyCycle);
  }

  public void openAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kReverse);
  }

  private TrapezoidProfileCommand getVelocityFromPosition(
      double positionSetpointGoal, double positionSetpointInit) {
    return new TrapezoidProfileCommand(
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                feedforward.maxAchievableVelocity(12, kA),
                feedforward.maxAchievableAcceleration(12, kV)),
            new TrapezoidProfile.State(positionSetpointGoal, 0),
            new TrapezoidProfile.State(positionSetpointInit, 0)),
        setpoint -> velocitySetpoint = setpoint.velocity,
        this);
  }

  private Command reachSideHeight(
      SparkMaxPIDController controller, RelativeEncoder encoder, double height) {
    return getVelocityFromPosition(height, encoder.getPosition())
        .andThen(
            new Command() {
              @Override
              public void initialize() {
                controller.setReference(
                    height,
                    ControlType.kPosition,
                    0,
                    feedforward.calculate(velocitySetpoint),
                    SparkMaxPIDController.ArbFFUnits.kVoltage);
              }

              @Override
              public void end(boolean interrupted) {
                controller.setReference(0, ControlType.kDutyCycle);
              }

              @Override
              public boolean isFinished() {
                double position = encoder.getPosition();
                return abs(height - position) <= THRESHOLD;
              }

              @Override
              public Set<Subsystem> getRequirements() {
                return Set.of(Climber.this);
              }
            });
  }

  private Command reachBothHeight(double height) {
    return reachSideHeight(leftController, leftEncoder, height)
        .alongWith(reachSideHeight(rightController, rightEncoder, height));
  }

  public Command upCommand() {
    return reachBothHeight(MAX_HEIGHT);
  }

  public Command downCommand() {
    return reachBothHeight(0);
  }

  public Command offCommand() {
    return new InstantCommand(() -> activateMotors(MotorMode.OFF));
  }

  public Command openAnglerCommand() {
    return new InstantCommand(this::openAngler, this);
  }

  public Command closeAnglerCommand() {
    return new InstantCommand(this::closeAngler, this);
  }

  public Command climbCommandGroup() {
    return upCommand() // TODO: Drive forwards after upCommand
        .andThen(downCommand())
        .andThen(closeAnglerCommand())
        .andThen(upCommand())
        .andThen(openAnglerCommand());
  }

  double getSpeed() {
    return leftMotor.get();
  }

  public Command climberManualCommand(DoubleSupplier motorSpeed, BooleanSupplier piston) {
    return new RunCommand(
        () -> {
          leftMotor.set(motorSpeed.getAsDouble());
          rightMotor.set(motorSpeed.getAsDouble());
          if (piston.getAsBoolean()) this.anglerPiston.toggle();
        },
        this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("leftHeight", leftEncoder::getPosition, null);
    builder.addDoubleProperty("rightHeight", rightEncoder::getPosition, null);
  }
}
