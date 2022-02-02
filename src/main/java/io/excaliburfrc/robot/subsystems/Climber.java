package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;
import static io.excaliburfrc.robot.Constants.ClimberConstants.kD;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final CANSparkMax motor =
      new CANSparkMax(ClimberConstants.CLIMBER_SPARKMAX, MotorType.kBrushless);
  private final DigitalInput sensor = new DigitalInput(ClimberConstants.SENSOR_CHANNEL);

  private final RelativeEncoder encoder = motor.getEncoder();

  private final SparkMaxPIDController controller = motor.getPIDController();

  public Climber() {
      ValidateREVCAN(
              // reset factory settings
              motor.restoreFactoryDefaults(),
              // set the motors to brake mode
              motor.setIdleMode(CANSparkMax.IdleMode.kBrake),
              motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
              motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
              // set up PID parameters
              controller.setFeedbackDevice(encoder),
              controller.setP(kP),
              controller.setI(kI),
              controller.setD(kD)
      );
  }

  @Override
  public void close() {
    anglerPiston.close();
    motor.close();
    sensor.close();
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

  public void activateLeader(MotorMode m) {
    motor.set(m.dutyCycle);
  }

  public void openAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public Command upCommand() {
    return new RunCommand(() -> activateLeader(MotorMode.UP), this)
        .withInterrupt(sensor::get)
        .andThen(offCommand());
  }

  public Command downCommand() {
    return new RunCommand(() -> activateLeader(MotorMode.DOWN), this)
        .withInterrupt(sensor::get)
        .andThen(offCommand());
  }

  public Command offCommand() {
    return new InstantCommand(() -> activateLeader(MotorMode.OFF));
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

  double _getSpeed() {
    return motor.get();
  }

  public Command climberManualCommand(DoubleSupplier motorSpeed, BooleanSupplier piston) {
    return new RunCommand(
        () -> {
          motor.set(motorSpeed.getAsDouble());
          if (piston.getAsBoolean()) anglerPiston.toggle();
        },
        this);
  }
}
