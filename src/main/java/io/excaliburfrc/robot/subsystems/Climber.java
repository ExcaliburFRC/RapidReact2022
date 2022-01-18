package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final CANSparkMax motor =
      new CANSparkMax(ClimberConstants.CLIMBER_SPARKMAX, MotorType.kBrushless);
  private final DigitalInput sensor = new DigitalInput(ClimberConstants.SENSOR_CHANNEL);

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
    return new InstantCommand(() -> openAngler(), this);
  }

  public Command closeAnglerCommand() {
    return new InstantCommand(() -> closeAngler(), this);
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
}
