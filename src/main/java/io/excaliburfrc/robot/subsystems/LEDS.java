package io.excaliburfrc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.LedsConstants;

public class LEDS extends SubsystemBase {
  private final PWM leds = new PWM(LedsConstants.LEDS_PORT);

  public enum LedMode {
    BLUE(0.87),
    RED(0.61),
    GREEN(0.73),
    YELLOW(0.67),
    RAINBOW(-0.97),
    OFF(0.99);

    LedMode(double c) {
      dutyCycle = c;
    }

    public final double dutyCycle;
  }

  public Command setColor(LedMode color) {
    return new RunCommand(() -> leds.setSpeed(color.dutyCycle), this);
  }

  public Command ledsOff() {
    return new RunCommand(() -> leds.setSpeed(LedMode.OFF.dutyCycle), this);
  }
}
