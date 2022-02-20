package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final ClimberSide left = new ClimberSide(LEFT_MOTOR_ID);
  private final ClimberSide right = new ClimberSide(ClimberConstants.RIGHT_MOTOR_ID);

  private final ElevatorFeedforward upFF = new ElevatorFeedforward(kS, MG, kV, kA);
  private final ElevatorFeedforward diagonalFF =
      new ElevatorFeedforward(kS, MG * Math.cos(ANGLE), kV, kA);

  private final TrapezoidProfile elevatorProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),
          new TrapezoidProfile.State(HEIGHT, 0), // The goal state
          new TrapezoidProfile.State(0, 0)); // The init state

  private class ClimberSide implements AutoCloseable {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController controller;

    public ClimberSide (int motorId) {
      motor = new CANSparkMax(motorId, MotorType.kBrushless);
      encoder = motor.getEncoder();
      controller = motor.getPIDController();

      ValidateREVCAN(
              // reset factory settings
              motor.restoreFactoryDefaults(),
              motor.restoreFactoryDefaults(),
              // set the motors to brake mode
              motor.setIdleMode(IdleMode.kBrake),
              motor.setIdleMode(IdleMode.kBrake),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
              // set up PID parameters
              controller.setFeedbackDevice(encoder),
              controller.setP(kP),
              controller.setI(kI),
              controller.setD(kD),
              controller.setFeedbackDevice(encoder),
              controller.setP(kP),
              controller.setI(kI),
              controller.setD(kD)
      );
    }

    public Command downCommand() {
      return new FunctionalCommand(
              () -> {},
              () -> motor.set(MotorMode.DOWN.dutyCycle),
              __ -> motor.set(MotorMode.OFF.dutyCycle),
              () -> encoder.getPosition() <= SAFETY_DISTANCE);
    }

    public void close() {
      motor.close();
    }

    public void set(double dutyCycle) {
      motor.set(dutyCycle);
    }

    public void setReference(double velocity, ControlType kVelocity, int i, double calculate, ArbFFUnits kVoltage) {
      controller.setReference(velocity, kVelocity, i, calculate, kVoltage);
    }
  }

  @Override
  public void close() {
    left.close();
    right.close();
    anglerPiston.close();
  }

  public enum MotorMode {
    OFF(0),
    UP(0.6),
    DOWN(-0.8);

    final double dutyCycle;

    MotorMode(double v) {
      dutyCycle = v;
    }
  }

  public void activateMotors(MotorMode m) {
    left.set(m.dutyCycle);
    right.set(m.dutyCycle);
  }

  public void openAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kReverse);
  }

  private void achieveState(TrapezoidProfile.State setpoint) {
    ElevatorFeedforward ff;
    if (anglerPiston.get() == DoubleSolenoid.Value.kForward) ff = diagonalFF;
    else ff = upFF;
    left.setReference(
        setpoint.velocity,
        ControlType.kVelocity,
        0,
        ff.calculate(setpoint.velocity),
        ArbFFUnits.kVoltage);
    right.setReference(
        setpoint.velocity,
        ControlType.kVelocity,
        0,
        ff.calculate(setpoint.velocity),
        ArbFFUnits.kVoltage);
  }

  private Command reachBarCommand(Consumer<TrapezoidProfile.State> toRun) {
    return new TrapezoidProfileCommand(elevatorProfile, toRun, this);
  }

  public Command openToFirstCommand() {
    return reachBarCommand(this::achieveState);
  }

  public Command openToSecondCommand() {
    return reachBarCommand(
        setpoint -> {
          achieveState(setpoint);
          if (setpoint.position >= HEIGHT_TO_OPEN_PISTON
              && anglerPiston.get() != DoubleSolenoid.Value.kForward) {
            anglerPiston.set(DoubleSolenoid.Value.kForward);
          }
        });
  }

  public Command raiseRobotCommand() {
    return left.downCommand().alongWith(right.downCommand());
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

  public Command climberManualCommand(DoubleSupplier motorSpeed, BooleanSupplier piston) {
    return new RunCommand(
        () -> {
          left.set(motorSpeed.getAsDouble());
          right.set(motorSpeed.getAsDouble());
          if (piston.getAsBoolean()) this.anglerPiston.toggle();
        },
        this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("leftHeight", left.encoder::getPosition, null);
    builder.addDoubleProperty("rightHeight", right.encoder::getPosition, null);
  }
}
