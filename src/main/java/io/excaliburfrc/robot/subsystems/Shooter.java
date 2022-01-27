package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leader =
      new CANSparkMax(ShooterConstants.LEADER_ID, MotorType.kBrushless);

  @SuppressWarnings("FieldCanBeLocal")
  private final CANSparkMax follower =
      new CANSparkMax(ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);

  private final Encoder encoder =
      new Encoder(ShooterConstants.ENCODER_A, ShooterConstants.ENCODER_B);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private final PIDController pid =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  private Mode controlMode = Mode.OFF;

  public Shooter() {
    ValidateREVCAN(
        // reset factory settings
        leader.restoreFactoryDefaults(),
        follower.restoreFactoryDefaults(),
        // set the motors to coast mode -- we don't want to break them!
        leader.setIdleMode(IdleMode.kCoast),
        follower.setIdleMode(IdleMode.kCoast),
        // have the leader send its applied output as frequently as possible,
        // to speed up follower response
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, minimal_FRAME_PERIOD),
        // other status frames can be reduced to almost never
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        // setup following
        follower.follow(leader));
  }

  public Command manualCommand(DoubleSupplier speed) {
    return new InstantCommand(() -> controlMode = Mode.MANUAL, this)
        .andThen(new RunCommand(() -> leader.set(speed.getAsDouble()), this));
  }

  public Command accelerateFenderCommand() {
    return accelerateToVelocityCommand(ShooterConstants.FENDER_SHOT_RPM);
  }

  private Command accelerateToVelocityCommand(double rpm) {
    return new StartEndCommand(() -> accelerate(rpm), this::release, this);
  }

  private void accelerate(double setpoint) {
    pid.setSetpoint(setpoint);
    controlMode = Mode.CLOSED_LOOP;
  }

  private void release() {
    pid.setSetpoint(0);
    controlMode = Mode.OFF;
  }

  private double x = 0;
  private double t = Timer.getFPGATimestamp() - 0.02;

  public double getVelocity() {
    double prevX = x;
    double prevT = t;

    x = encoder.getDistance();
    t = Timer.getFPGATimestamp();

    double dx = x - prevX;
    double dt = t - prevT;

    return dx / dt;
  }

  @Override
  public void periodic() {
    double velocity = getVelocity();

    // TODO: add telemetry/logging

    switch (controlMode) {
      case OFF:
        leader.set(0);
        break;

      case MANUAL:
        // no-op
        // if running manual, the command is responsible for directly setting motor speed
        break;

      case CLOSED_LOOP:
        double ffOutput = feedforward.calculate(pid.getSetpoint());
        double pidOutput = pid.calculate(velocity);
        leader.set(pidOutput + ffOutput);
        break;
    }
  }

  private enum Mode {
    OFF,
    MANUAL,
    CLOSED_LOOP
  }
}
