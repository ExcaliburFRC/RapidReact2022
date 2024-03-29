package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.lib.TriggerUtils.Falling;
import static io.excaliburfrc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.excaliburfrc.lib.RepeatingCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final AtomicInteger ballCount = new AtomicInteger(1);
  private final AtomicBoolean allow = new AtomicBoolean(false);

  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  final Trigger intakeBallTrigger =
      new Trigger(() -> intakeSensor.getProximity() > COLOR_LIMIT).debounce(0.1);
  private final Ultrasonic upperSensor = new Ultrasonic(UPPER_PING, UPPER_ECHO);
  final Trigger upperBallTrigger = new Trigger(() -> upperSensor.getRangeMM() < SONIC_LIMIT).debounce(0.15);
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

  public int getBallCount() {
    return ballCount.get();
  }

  public Command openPiston() {
    return new InstantCommand(() -> intakePiston.set(Value.kForward), this);
  }

  public Command closePiston() {
    return new InstantCommand(() -> intakePiston.set(Value.kReverse), this);
  }

  public Command pullIntoIntake() {
    return new StartEndCommand(
            () -> intakeMotor.set(Speeds.intakeInDutyCycle), () -> intakeMotor.set(0), this)
        .until(intakeBallTrigger)
        .andThen(ballCount::incrementAndGet);
  }

  public Command pullIntoUpper() {
    return new StartEndCommand(
            () -> {
              intakeMotor.set(Speeds.intakeInDutyCycle);
              upperMotor.set(Speeds.upperInDutyCycle);
            },
            () -> {
              intakeMotor.set(0);
              upperMotor.set(0);
            },
            this)
        .until(upperBallTrigger);
  }

  public Command intakeTick() {
    return new StartEndCommand(
            () -> intakeMotor.set(Speeds.intakeInDutyCycle), () -> intakeMotor.set(0))
        .until(intakeBallTrigger.negate());
  }

  public boolean intakeFull(){
    return upperBallTrigger.and(intakeBallTrigger).get();
  }

  public Command pullIntoShooter(Trigger ballShotTrigger) {
    return new StartEndCommand(
            () -> upperMotor.set(Speeds.upperShootDutyCycle), () -> upperMotor.set(0), this)
        .until(ballShotTrigger)
        .andThen(ballCount::decrementAndGet);
  }

  public Command ejectFromIntake() {
    return new ConditionalCommand(
        new FunctionalCommand(
                () -> {},
                () -> intakeMotor.set(Speeds.intakeEjectDutyCycle),
                interrupted -> intakeMotor.set(0),
                intakeBallTrigger.negate().debounce(0.2),
                this)
            .andThen(ballCount::decrementAndGet),
        new PrintCommand("did nothing"),
        intakeBallTrigger);
  }

  public Command rawEject() {
    return openPiston()
          .andThen(new WaitCommand(0.3))
          .andThen(
          new StartEndCommand(
            () -> {
              intakePiston.set(Value.kReverse);
              upperMotor.set(Speeds.upperEjectDutyCycle);
              intakeMotor.set(Speeds.intakeEjectDutyCycle);
            },
            () -> {
              upperMotor.set(0);
              intakeMotor.set(0);
            },
            this))
        .alongWith(
            new RepeatingCommand(
                new SequentialCommandGroup(new WaitUntilCommand(Falling(intakeBallTrigger)))))
        .andThen(() -> ballCount.set(0));
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
            intakePiston.set(isOpen() ? Value.kReverse : Value.kForward);
        },
        this);
  }

  @Deprecated
  public Command blindShootBallCommand() {
    return new FunctionalCommand(
            // init
            () -> {},
            // exe
            () -> upperMotor.set(Speeds.upperShootDutyCycle),
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

  public boolean isIntakeMotorRunning(){
    return intakeMotor.get() > 0.1;
  }

  boolean isOurColor() {
    var red = intakeSensor.getRed();
    var blue = intakeSensor.getBlue();
    boolean result = false;
    switch (DriverStation.getAlliance()) {
      case Red:
        result = red > blue;
        DriverStation.reportWarning("Our Alliance color: " + result + ". red: " + red + ", blue: " + blue, false);
        break;
      case Blue:
        result = red < blue;
        DriverStation.reportWarning("Our Alliance color: " + result + ". red: " + red + ", blue: " + blue, false);
        break;
      default:
        DriverStation.reportError("DS Alliance color is invalid!", false);
    }
    return result || allow.get();
  }

  public void resetBallCounter(int n) {
    ballCount.set(n);
  }

  @Override
  public void close() {
    this.intakeMotor.close();
    this.upperMotor.close();
    this.upperSensor.close();
  }

  public Command allowCommand() {
    return new StartEndCommand(() -> allow.set(true), () -> allow.set(false));
  }

  private enum Speeds {
    ;
    static final double intakeEjectDutyCycle = -0.4;
    static final double upperEjectDutyCycle = -0.2;
    static final double upperShootDutyCycle = 0.6;

    static final double intakeInDutyCycle = 0.3;
    static final double upperInDutyCycle = 0.1;
  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    //    SendableRegistry.remove(upperSensor);
    SendableRegistry.remove(intakePiston);

    builder.addBooleanProperty("Allow", allow::get, allow::set);
    builder.addBooleanProperty("Intake Cargo", intakeBallTrigger, null);
    builder.addBooleanProperty("Upper Cargo", upperBallTrigger, null);
    builder.addDoubleProperty("Cargo Count", ballCount::get, null);
    builder.addBooleanProperty("Intake Piston", this::isOpen, null);
    builder.addDoubleProperty("Ultrasonic range", upperSensor::getRangeMM, null);
  }

  public boolean isOpen() {
    return intakePiston.get() == Value.kForward;
  }

  public boolean isEmpty() {
    return ballCount.get() < 1;
  }
}
