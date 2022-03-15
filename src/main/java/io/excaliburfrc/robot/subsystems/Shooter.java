package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.robot.Constants.ShooterConstants.RPS_DROP_ON_SHOOT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.robot.Constants.ShooterConstants;
import java.util.concurrent.atomic.AtomicInteger;

public class Shooter extends SubsystemBase {
  private final AtomicInteger currentTarget =
      new AtomicInteger((int) ShooterConstants.FENDER_SHOT_RPM);
  private final CANSparkMax leader =
      new CANSparkMax(ShooterConstants.LEADER_ID, MotorType.kBrushless);

  @SuppressWarnings("FieldCanBeLocal")
  private final CANSparkMax follower =
      new CANSparkMax(ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);

  private final Encoder encoder =
      new Encoder(ShooterConstants.ENCODER_A, ShooterConstants.ENCODER_B);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private final PIDController pid = new PIDController(ShooterConstants.kP, 0, 0);

  private Mode controlMode = Mode.OFF;
  private double velocity = 0;

  private double lastVelocity = velocity;
  private final Trigger ballShotTrigger =
      new Trigger(() -> pid.getSetpoint() > 0 && lastVelocity - velocity > RPS_DROP_ON_SHOOT);

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
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DEFAULT),
        // other status frames can be reduced to almost never
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        leader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DO_NOT_SEND),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND),
        leader.setSmartCurrentLimit(80),
        // setup following
        follower.follow(leader));
    leader.setInverted(true);

    encoder.setDistancePerPulse(ShooterConstants.ROTATIONS_PER_PULSE);
  }

  public Command manualCommand() {
    return new FunctionalCommand(
        () -> controlMode = Mode.MANUAL,
        () -> leader.set(0.6738),
        __ -> leader.set(0),
        () -> false,
        this);
  }

  public Command accelerateFenderCommand() {
    return new StartEndCommand(() -> accelerate(currentTarget.get()), this::release, this);
  }

  public double getVelocity() {
    return velocity;
  }

  final Trigger atTargetVelocity = new Trigger(this::isAtTargetVelocity).debounce(0.1);

  public boolean isAtTargetVelocity() {
    return Math.abs(getVelocity() - pid.getSetpoint()) < ShooterConstants.TOLERANCE;
  }

  public Command ejectLow() {
    return new StartEndCommand(
        () -> {
          controlMode = Mode.MANUAL;
          leader.set(0.25);
        },
        () -> {
          controlMode = Mode.OFF;
          leader.set(0);
        },
        this);
  }

  public Command incrementTarget(int diff) {
    return new InstantCommand(() -> currentTarget.addAndGet(diff));
  }

  private void accelerate(@SuppressWarnings("SameParameterValue") double setpoint) {
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

  @Override
  public void periodic() {
    lastVelocity = velocity;
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
        leader.setVoltage(pidOutput + ffOutput);
        break;
    }

    SmartDashboard.putBoolean("ball shot trigger", ballShotTrigger.get());
    SmartDashboard.putNumber("diff", lastVelocity - velocity);
    DriverStation.reportWarning("vel " + velocity, false);
  }

  private enum Mode {
    OFF,
    MANUAL,
    CLOSED_LOOP
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    LiveWindow.disableTelemetry(encoder);
    LiveWindow.disableTelemetry(pid);

    builder.setSmartDashboardType("Subsystem");
    builder.addDoubleProperty("rps", currentTarget::get, null);
    builder.addDoubleProperty("velocity", this::getVelocity, null);
    builder.addDoubleProperty("targetVelocity", pid::getSetpoint, null);
    builder.addDoubleProperty("control effort", leader::getAppliedOutput, null);
    builder.addDoubleProperty("control current", leader::getOutputCurrent, null);
    builder.addBooleanProperty("isAtReference", this::isAtTargetVelocity, null);
  }
}
