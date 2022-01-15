package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.ClimberConstants.FORWARD_CHANNEL,
          Constants.ClimberConstants.REVERSE_CHANNEL);
  private CANSparkMax motor =
      new CANSparkMax(
          Constants.ClimberConstants.CLIMBER_SPARKMAX, CANSparkMaxLowLevel.MotorType.kBrushless);
  private DigitalInput sensor = new DigitalInput(Constants.ClimberConstants.SENSOR_CHANNEL);

  private enum MotorMode {
    OFF(0),
    UP(0.6),
    DOWN(-0.4);

    private final double dutyCycle;

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
            .andThen(upCommand())
            .andThen(openAnglerCommand())
            .andThen(closeAnglerCommand());
  }
}
