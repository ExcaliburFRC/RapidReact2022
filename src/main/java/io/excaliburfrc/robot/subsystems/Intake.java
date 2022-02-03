package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.IntakeConstants.*;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final AtomicInteger ballCount = new AtomicInteger(0);

  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Trigger intakeBallTrigger =
      new Trigger(() -> intakeSensor.getProximity() < COLOR_LIMIT);
  private final Ultrasonic upperSensor = new Ultrasonic(PING, ECHO);
  private final Trigger upperBallTrigger =
      new Trigger(() -> upperSensor.getRangeMM() < SONIC_LIMIT);
  private final DoubleSolenoid intakePiston =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FWD_CHANNEL, REV_CHANNEL);
  private final ConditionalCommand automaticCommand;

  public Intake() {
    ValidateREVCAN(
        // reset factory settings
        intakeMotor.restoreFactoryDefaults(),
        upperMotor.restoreFactoryDefaults(),
        // set the motors to break mode -- we don't want cargo to be able to escape
        upperMotor.setIdleMode(IdleMode.kBrake),
        intakeMotor.setIdleMode(IdleMode.kBrake),
        // decrease status frames -- nothing is of interest here
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD));

    automaticCommand =
        new ConditionalCommand(
            // increment ball count; input until upper sensor detects a ball
            new FunctionalCommand(
                // init
                ballCount::incrementAndGet,
                // exe
                () -> {
                  intakeMotor.set(Speeds.intakeInDutyCycle);
                  upperMotor.set(Speeds.upperInDutyCycle);
                },
                // end
                _interrupted -> {
                  intakeMotor.set(0);
                  upperMotor.set(0);
                },
                // isFinished
                upperBallTrigger,
                this),

            // output sets motor until ball entry sensor no longer sees a ball
            new FunctionalCommand(
                // init
                () -> {},
                // exe
                () -> intakeMotor.set(Speeds.ejectDutyCycle),
                // end
                _interrupted -> intakeMotor.set(0),
                // isFinished
                intakeBallTrigger.negate()),
            // decides by ball color
            this::isOurColor);
    // schedule the command whenever the entry sensor newly activates ...
    intakeBallTrigger.whenActive(automaticCommand);

    // update the counter whenever we shoot a ball
    upperBallTrigger.whenInactive(ballCount::decrementAndGet, this);
    // and report if we pass the limit
    new Trigger(() -> ballCount.get() > MAX_BALLS)
        .whenActive(() -> DriverStation.reportWarning("Too many Cargo on robot!", false));
  }

  public Command manualCommand(
      DoubleSupplier intake, DoubleSupplier upper, BooleanSupplier pistonState) {
    return new RunCommand(
        () -> {
          intakeMotor.set(intake.getAsDouble());
          upperMotor.set(upper.getAsDouble());
          if (pistonState.getAsBoolean()) intakePiston.toggle();
        },
        this);
  }

  /** Shoot *one* ball; will end after a ball is shot. */
  public Command shootBallCommand() {
    return new FunctionalCommand(
        // init
        () -> {},
        // exe
        () -> {
          upperMotor.set(Speeds.upperShootDutyCycle);
          intakeMotor.set(Speeds.intakeShootDutyCycle);
        },
        // end
        _interrupted -> {
          upperMotor.set(0);
          intakeMotor.set(0);
        },
        // isFinished
        // stop after we've shot a ball
        upperBallTrigger.negate());
  }

  private boolean isOurColor() {
    switch (DriverStation.getAlliance()) {
      case Red:
        return intakeSensor.getRed() > RED_THRESHOLD;
      case Blue:
        return intakeSensor.getBlue() > BLUE_THRESHOLD;
    }
    DriverStation.reportError("DS Alliance color is invalid!", false);
    return false;
  }

  @Override
  public void close() {
    this.intakeMotor.close();
    this.upperMotor.close();
    this.upperSensor.close();
  }

  private enum Speeds {
    ;
    static final double ejectDutyCycle = -0.6;

    static final double intakeInDutyCycle = 0.3;
    static final double upperInDutyCycle = 0.3;

    static final double intakeShootDutyCycle = 0.4;
    static final double upperShootDutyCycle = 0.6;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Intake Cargo", intakeBallTrigger, null);
    builder.addBooleanProperty("Upper Cargo", upperBallTrigger, null);
    builder.addDoubleProperty("Cargo Count", ballCount::get, null);

    builder.addBooleanProperty("Automatic Active", automaticCommand::isScheduled, null);
  }
}
