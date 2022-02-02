package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.LedsConstants.LEDS_PORT;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private static final PWM leds = new PWM(LEDS_PORT);

  public enum LedMode {
    BLUE(0.87),
    RED(0.61),
    GREEN(0.73),
    YELLOW(0.67),
    RAINBOW(-0.97),
    WHITE(0.93),
    PINK(0.57),
    GOLD(0.67),
    BLACK(0.99),
    OFF(0.99);

    LedMode(double c) {
      dutyCycle = c;
    }

    public final double dutyCycle;
  }

  public static PWM getInstance() {
    return leds;
  }

  public Command setColorCommand(LedMode color) {
    return new RunCommand(() -> leds.setSpeed(color.dutyCycle), this);
  }

  public Command ledsOffCommand() {
    return new RunCommand(() -> leds.setSpeed(LedMode.OFF.dutyCycle), this);
  }
}
