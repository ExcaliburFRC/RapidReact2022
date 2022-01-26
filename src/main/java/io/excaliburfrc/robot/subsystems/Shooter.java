package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leader =
      new CANSparkMax(ShooterConstants.LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);
  private final Encoder encoder =
      new Encoder(ShooterConstants.ENCODER_A, ShooterConstants.ENCODER_B);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
  private final PIDController pid =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  private boolean isControl = false;

  public Shooter() {
    follower.follow(leader);
  }

  public Command manualCommand(DoubleSupplier speed) {
    return new RunCommand(() -> leader.set(speed.getAsDouble() * 0.5), this);
  }

  public Command accelerateToVelocityCommand(double rpm) {
    return new StartEndCommand(() -> accelerate(rpm), this::release);
  }

  private void accelerate(double setpoint) {
    pid.setSetpoint(setpoint);
    isControl = true;
  }

  private void release() {
    pid.setSetpoint(0); // for safety reasons
    isControl = false;
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
    if (isControl) {
      double ffOutput = feedforward.calculate(pid.getSetpoint());
      double pidOutput = pid.calculate(getVelocity());
      leader.set(pidOutput + ffOutput);
    }
  }
}
