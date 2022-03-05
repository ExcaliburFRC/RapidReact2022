package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final AtomicInteger climbStage = new AtomicInteger(0);

  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final ClimberSide left = new ClimberSide(LEFT_MOTOR_ID, false);
  private final ClimberSide right = new ClimberSide(RIGHT_MOTOR_ID, true);

  private final ElevatorFeedforward upFF = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ElevatorFeedforward diagonalFF =
      new ElevatorFeedforward(kS, kG * Math.cos(ANGLE), kV, kA);

  private final TrapezoidProfile elevatorProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),
          new TrapezoidProfile.State(OPEN_HEIGHT, 0), // The goal state
          new TrapezoidProfile.State(0, 0)); // The init state

  private static class ClimberSide implements AutoCloseable {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController controller;

    public ClimberSide(int motorId, boolean isMotorReversed) {
      motor = new CANSparkMax(motorId, MotorType.kBrushless);
      encoder = motor.getEncoder();
      controller = motor.getPIDController();

      ValidateREVCAN(
          // reset factory settings
          motor.restoreFactoryDefaults(),
          // set the motors to brake mode
          motor.setIdleMode(IdleMode.kBrake),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DEFAULT),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DEFAULT),
          motor.setSoftLimit(SoftLimitDirection.kReverse, 0),
          motor.enableSoftLimit(SoftLimitDirection.kReverse, true),
          motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.FORWARD_SOFT_LIMIT),
          motor.enableSoftLimit(SoftLimitDirection.kForward, true),
          // set up PID parameters
          controller.setFeedbackDevice(encoder),
          controller.setP(kP),
          controller.setI(kI),
          controller.setD(kD));
      motor.setInverted(isMotorReversed);

      resetPosition();
    }

    public Command pullUpCommand() {
      return new FunctionalCommand(
          () -> {},
          () -> motor.set(-OPEN_LOOP_CLIMB_DUTY_CYCLE),
          __ -> motor.set(0),
          () -> encoder.getPosition() <= CLOSED_HEIGHT);
    }

    public Command pullUpHalfCommand() {
      return new FunctionalCommand(
          () -> {},
          () -> motor.set(-OPEN_LOOP_CLIMB_DUTY_CYCLE),
          __ -> motor.set(0),
          () -> encoder.getPosition() <= CLOSED_HEIGHT);
    }

    public Command openFullyCommand() {
      return new FunctionalCommand(
          () -> {},
          () -> motor.set(OPEN_LOOP_CLIMB_DUTY_CYCLE),
          __ -> motor.set(0),
          () -> encoder.getPosition() >= OPEN_HEIGHT);
    }

    public void resetPosition() {
      encoder.setPosition(0);
    }

    public void close() {
      motor.close();
    }

    public void setDutyCycle(double dutyCycle) {
      motor.set(dutyCycle);
    }

    public Command manualCommand(BooleanSupplier up, BooleanSupplier down) {
      return new FunctionalCommand(
          () -> {},
          () -> {
            if (up.getAsBoolean()) motor.set(0.1);
            else if (down.getAsBoolean()) motor.set(-0.1);
            else motor.set(0);
          },
          __ -> motor.set(0),
          () -> false);
    }

    //    public void setReference(double velocity, double ff) {
    //      controller.setReference(velocity, ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);
    //    }

    public double getHeight() {
      return encoder.getPosition();
    }

    /** Disables soft limits for the duration of this command, and then resets the encoder. */
    public Command disableAndResetSoftLimits() {
      return new StartEndCommand(
          () -> {
            motor.enableSoftLimit(SoftLimitDirection.kForward, false);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
          },
          () -> {
            encoder.setPosition(0);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
          });
    }
  }

  @Override
  public void close() {
    left.close();
    right.close();
    anglerPiston.close();
  }

  //  @Deprecated
  //  private void achieveState(TrapezoidProfile.State setpoint) {
  //    ElevatorFeedforward ff;
  //    if (anglerPiston.get() == DoubleSolenoid.Value.kForward) ff = diagonalFF;
  //    else ff = upFF;
  //    //    left.setReference(setpoint.velocity, ff.calculate(setpoint.velocity));
  //    right.setReference(setpoint.velocity, ff.calculate(setpoint.velocity));
  //  }
  //
  //  private Command reachBarCommand(Consumer<TrapezoidProfile.State> toRun) {
  //    return new TrapezoidProfileCommand(elevatorProfile, toRun, this);
  //  }

  public Command openFullyCommand() {
    return new ParallelCommandGroup(left.openFullyCommand(), right.openFullyCommand());
  }

  public Command pullUpRobotCommand() {
    return left.pullUpCommand().alongWith(right.pullUpCommand());
  }

  public Command pullUpHalfRobotCommand() {
    return left.pullUpHalfCommand().alongWith(right.pullUpHalfCommand());
  }

  public Command offCommand() {
    return new InstantCommand(
        () -> {
          left.setDutyCycle(0);
          right.setDutyCycle(0);
        });
  }

  public Command openAnglerCommand() {
    return new InstantCommand(
        () -> {
          anglerPiston.set(ANGLED);
        },
        this);
  }

  public Command straightenAnglerCommand() {
    return new InstantCommand(
        () -> {
          anglerPiston.set(STRAIGHT);
        },
        this);
  }

  public Command climbSeries(
      BooleanSupplier openFirst,
      BooleanSupplier closeFirst,
      BooleanSupplier openSecond,
      BooleanSupplier angleSecond,
      BooleanSupplier closeSecond,
      BooleanSupplier closeAngle) {
    return new SequentialCommandGroup(
            new PrintCommand("Started Climb Series"),
            new WaitUntilCommand(openFirst),
            new PrintCommand("Started Climb Series 2"),
            openFullyCommand(),
            new WaitUntilCommand(closeFirst),
            new PrintCommand("Started Climb Series 3"),
            pullUpRobotCommand(),
            new WaitUntilCommand(openSecond),
            new PrintCommand("Started Climb Series 4"),
            openFullyCommand(),
            new WaitUntilCommand(angleSecond),
            new PrintCommand("Started Climb Series"),
            openAnglerCommand(),
            new WaitUntilCommand(() -> !angleSecond.getAsBoolean()),
            new WaitUntilCommand(closeSecond),
            new PrintCommand("Started Climb Series"),
            pullUpHalfRobotCommand(),
            new WaitUntilCommand(closeAngle),
            new PrintCommand("Started Climb Series"),
            straightenAnglerCommand())
        .withName("climb series");
  }

  public Command climberManualCommand(
      BooleanSupplier leftUp,
      BooleanSupplier leftDown,
      BooleanSupplier rightUp,
      BooleanSupplier rightDown,
      BooleanSupplier pistonAngled,
      BooleanSupplier pistonStraight) {
    return new ParallelCommandGroup(
        left.manualCommand(leftUp, leftDown),
        right.manualCommand(rightUp, rightDown),
        new FunctionalCommand(
            () -> {},
            () -> {
              if (pistonAngled.getAsBoolean()) anglerPiston.set(ANGLED);
              if (pistonStraight.getAsBoolean()) anglerPiston.set(STRAIGHT);
            },
            __ -> {},
            () -> false));
  }

  public Command disableSoftLimits() {
    return left.disableAndResetSoftLimits().alongWith(right.disableAndResetSoftLimits());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    builder.addDoubleProperty("climb stage", climbStage::get, v -> climbStage.set((int) v));
    builder.addDoubleProperty("leftHeight", left::getHeight, null);
    builder.addDoubleProperty("rightHeight", right::getHeight, null);

    builder.addDoubleProperty("d/rightControl", right.motor::get, null);
  }
}
