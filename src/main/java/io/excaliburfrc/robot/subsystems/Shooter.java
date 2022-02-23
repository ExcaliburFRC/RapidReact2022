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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ShooterConstants;
import java.util.function.BooleanSupplier;
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
  private double velocity = 0;

  public Shooter() {
    leader.setInverted(true);

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
    return new FunctionalCommand(
        () -> controlMode = Mode.MANUAL,
        () -> leader.set(speed.getAsDouble()),
        __ -> leader.set(0),
        () -> false,
        this);
  }

  public Command accelerateFenderCommand() {
    return new StartEndCommand(
        () -> accelerate(ShooterConstants.FENDER_SHOT_RPM), this::release, this);
  }

  public double getVelocity() {
    return velocity;
  }

  public boolean isAtTargetVelocity() {
    return controlMode == Mode.CLOSED_LOOP
        && Math.abs(getVelocity() - pid.getSetpoint()) < ShooterConstants.TOLERANCE;
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

  private void updateVelocity() {
    double prevX = x;
    double prevT = t;

    x = encoder.getDistance();
    t = Timer.getFPGATimestamp();

    double dx = x - prevX;
    double dt = t - prevT;

    velocity = dx / dt;
  }

  public void setMotor(DoubleSupplier speed){
    leader.set(speed.getAsDouble());
  }

  @Override
  public void periodic() {
    updateVelocity();

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

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("targetVelocity", pid::getSetpoint, null);
    builder.addBooleanProperty("isAtReference", this::isAtTargetVelocity, null);
  }
}
