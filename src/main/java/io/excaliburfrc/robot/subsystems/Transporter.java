package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.robot.Constants.TranporterConstants;
import java.util.concurrent.atomic.AtomicInteger;

public class Transporter extends SubsystemBase {
  private final AtomicInteger ballCount = new AtomicInteger(0);

  private final CANSparkMax lower =
      new CANSparkMax(TranporterConstants.LOWER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax upper =
      new CANSparkMax(TranporterConstants.UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 color = new ColorSensorV3(I2C.Port.kMXP);
  private final Ultrasonic sonic =
      new Ultrasonic(TranporterConstants.PING, TranporterConstants.ECHO);

  public Transporter() {
    Trigger ballStopTrigger =
        new Trigger(() -> sonic.getRangeMM() < TranporterConstants.SONIC_LIMIT);
    Trigger ballEntryTrigger =
        new Trigger(() -> color.getProximity() < TranporterConstants.COLOR_LIMIT);

    // schedule the command whenever the entry sensor newly activates ...
    ballEntryTrigger.whenActive(
        new ConditionalCommand(
            // increment ball count; input until upper sensor detects a ball
            new InstantCommand(ballCount::incrementAndGet, this)
                .andThen(
                    new RunCommand(
                        () -> {
                          lower.set(Mode.IN.lower);
                          upper.set(Mode.IN.upper);
                          SmartDashboard.putNumber("Ball Counter", getBallCount());
                        },
                        this))
                .withInterrupt(ballStopTrigger)
                .andThen(
                    new InstantCommand(
                        () -> {
                          lower.set(Mode.OFF.lower);
                          upper.set(Mode.OFF.upper);
                        }))
                .withName("in"),

            // output sets motor until ball entry sensor no longer sees a ball
            new RunCommand(() -> lower.set(Mode.OUT.lower), this)
                .withInterrupt(ballEntryTrigger.negate())
                .andThen(new InstantCommand(() -> lower.set(Mode.OFF.lower)))
                .withName("out"),
            // decides by ball color
            this::isOurColor));

    // update the counter whenever we shoot a ball
    ballStopTrigger.whenInactive(ballCount::decrementAndGet, this);
    // and report if we pass the limit
    new Trigger(() -> ballCount.get() > TranporterConstants.MAX_BALLS)
        .whenActive(() -> DriverStation.reportWarning("Too many Cargo on robot!", false));
  }

  public int getBallCount() {
    return ballCount.get();
  }

  public boolean isOurColor() {
    switch (DriverStation.getAlliance()) {
      case Red:
        return color.getRed() > TranporterConstants.RED_THRESHOLD;
      case Blue:
        return color.getBlue() > TranporterConstants.BLUE_THRESHOLD;
    }
    DriverStation.reportError("DS Alliance color is invalid!", false);
    return false;
  }

  private enum Mode {
    // (lower, upper)
    SHOOT(0.4, 0.4),
    IN(0.3, 0),
    OUT(-0.4, 0),
    OFF(0, 0);
    public final double lower, upper;

    Mode(double lr, double ur) {
      lower = lr;
      upper = ur;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Cargo OverLoad", getBallCount() > TranporterConstants.MAX_BALLS);
  }
}
