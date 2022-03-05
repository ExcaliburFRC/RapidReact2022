package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final AtomicInteger ballCount = new AtomicInteger(0);
  private final AtomicBoolean allow = new AtomicBoolean(false);

  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Trigger intakeBallTrigger =
      new Trigger(() -> intakeSensor.getProximity() > COLOR_LIMIT);
  private final Ultrasonic upperSensor = new Ultrasonic(PING, ECHO);
  private final Trigger upperBallTrigger =
      new Trigger(() -> upperSensor.getRangeMM() < SONIC_LIMIT);
  private final DoubleSolenoid intakePiston =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FWD_CHANNEL, REV_CHANNEL);

  public Intake() {
    ValidateREVCAN(
        // reset factory settings
        intakeMotor.restoreFactoryDefaults(),
        upperMotor.restoreFactoryDefaults(),
        // set the motors to break mode -- we don't want cargo to be able to escape
        upperMotor.setIdleMode(IdleMode.kBrake),
        intakeMotor.setIdleMode(IdleMode.kBrake),
        // decrease status frames -- nothing is of interest here
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DO_NOT_SEND),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DO_NOT_SEND),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DO_NOT_SEND));

    // report if we pass the limit
    new Trigger(() -> ballCount.get() > MAX_BALLS)
        .whenActive(() -> DriverStation.reportWarning("Too many Cargo on robot!", false));

    intakeMotor.setInverted(true);
    upperMotor.setInverted(true);
    Ultrasonic.setAutomaticMode(true);
  }

  public Command intakeBallCommand() {
    return new FunctionalCommand(
            () -> intakePiston.set(Value.kForward),
            () -> intakeMotor.set(Speeds.intakeInDutyCycle),
            __ -> intakeMotor.set(0),
            intakeBallTrigger)
        .andThen(
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
                    __ -> {
                      intakeMotor.set(0);
                      upperMotor.set(0);
                      //                      intakePiston.set(Value.kReverse);
                    },
                    // isFinished
                    upperBallTrigger,
                    this),

                // output sets motor until ball entry sensor no longer sees a ball
                new FunctionalCommand(
                        // init
                        () -> {},
                        // exe
                        () -> {
                          intakeMotor.set(Speeds.ejectDutyCycle);
                          upperMotor.set(Speeds.ejectDutyCycle);
                        },
                        // end
                        __ -> {
                          intakeMotor.set(0);
                          upperMotor.set(0);
                          //                          intakePiston.set(Value.kReverse);
                        },
                        // isFinished
                        intakeBallTrigger.negate())
                    .andThen(
                        new StartEndCommand(
                                () -> intakeMotor.set(Speeds.ejectDutyCycle),
                                () -> intakeMotor.set(0))
                            .withTimeout(0.5)),
                // decides by ball color
                this::isOurColor));
  }

  public Command manualCommand(
      BooleanSupplier intakeIn,
      BooleanSupplier intakeOut,
      BooleanSupplier upperIn,
      BooleanSupplier upperOut,
      BooleanSupplier pistonState) {
    return new RunCommand(
        () -> {
          if (intakeIn.getAsBoolean()) intakeMotor.set(Speeds.intakeInDutyCycle);
          else if (intakeOut.getAsBoolean()) intakeMotor.set(-Speeds.intakeInDutyCycle);
          else intakeMotor.set(0);
          if (upperIn.getAsBoolean()) upperMotor.set(Speeds.upperShootDutyCycle);
          else if (upperOut.getAsBoolean()) upperMotor.set(-Speeds.upperInDutyCycle);
          else upperMotor.set(0);
          if (pistonState.getAsBoolean())
            intakePiston.set(
                intakePiston.get() == Value.kForward ? Value.kReverse : Value.kForward);
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
          if (!_interrupted) ballCount.decrementAndGet();
          upperMotor.set(0);
          intakeMotor.set(0);
        },
        // isFinished
        // stop after we've shot a ball
        upperBallTrigger.negate());
  }

  public Command blindShootBallCommand() {
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
            () -> false)
        .withTimeout(0.1);
  }

  private boolean isOurColor() {
    var red = intakeSensor.getRed();
    var blue = intakeSensor.getBlue();
    boolean result = false;
    switch (DriverStation.getAlliance()) {
      case Red:
        DriverStation.reportWarning("RED: " + red + ", " + blue, false);
        result = red > blue;
        break;
      case Blue:
        DriverStation.reportWarning("BLUE: " + red + ", " + blue, false);
        result = red < blue;
        break;
      default:
        DriverStation.reportError("DS Alliance color is invalid!", false);
    }
    return result || allow.get();
  }

  @Override
  public void close() {
    this.intakeMotor.close();
    this.upperMotor.close();
    this.upperSensor.close();
  }

  public Command ejectCommand() {
    return new StartEndCommand(
        () -> {
          this.intakeMotor.set(Speeds.ejectDutyCycle);
          this.upperMotor.set(Speeds.ejectDutyCycle);
        },
        () -> {
          this.intakeMotor.set(0);
          this.upperMotor.set(0);
        },
        this);
  }

  public Command allowCommand() {
    return new StartEndCommand(() -> allow.set(true), () -> allow.set(false));
  }

  private enum Speeds {
    ;
    static final double ejectDutyCycle = -0.6;

    static final double intakeInDutyCycle = 0.3;
    static final double upperInDutyCycle = 0.1;

    static final double intakeShootDutyCycle = 0.4;
    static final double upperShootDutyCycle = 0.7;
  }

  public Command setPistonCommand(Value val) {
    return new InstantCommand(() -> intakePiston.set(val));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");
    builder.addBooleanProperty("Allow", allow::get, allow::set);
    builder.addBooleanProperty("Intake Cargo", intakeBallTrigger, null);
    builder.addBooleanProperty("Upper Cargo", upperBallTrigger, null);
    builder.addDoubleProperty("Cargo Count", ballCount::get, null);
    builder.addStringProperty("Intake Piston", intakePiston.get()::toString, null);
  }

  public boolean isEmpty() {
    return ballCount.get() < 1;
  }
}
