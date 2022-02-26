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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final AtomicInteger ballCount = new AtomicInteger(0);

  private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax transporterMotor =
      new CANSparkMax(UPPER_MOTOR_ID, MotorType.kBrushless);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Trigger intakeBallTrigger =
      new Trigger(() -> intakeSensor.getProximity() < COLOR_LIMIT);
  private final Ultrasonic upperSensor = new Ultrasonic(PING, ECHO);
  private final Trigger upperBallTrigger =
      new Trigger(() -> upperSensor.getRangeMM() < SONIC_LIMIT);
  private final DoubleSolenoid intakePiston =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, FWD_CHANNEL, REV_CHANNEL);

  public Intake() {
    ValidateREVCAN(
        // reset factory settings
        intakeMotor.restoreFactoryDefaults(),
        transporterMotor.restoreFactoryDefaults(),
        // set the motors to break mode -- we don't want cargo to be able to escape
        transporterMotor.setIdleMode(IdleMode.kBrake),
        intakeMotor.setIdleMode(IdleMode.kBrake),
        // decrease status frames -- nothing is of interest here
        transporterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        transporterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        transporterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD));
    // update the counter whenever we shoot a ball
    upperBallTrigger.whenInactive(ballCount::decrementAndGet, this);
    // and report if we pass the limit
    new Trigger(() -> ballCount.get() > MAX_BALLS)
        .whenActive(() -> DriverStation.reportWarning("Too many Cargo on robot!", false));

    upperSensor.setEnabled(true);
  }

  public Command intakeBallCommand() {
    return new FunctionalCommand(
            () -> intakePiston.set(DoubleSolenoid.Value.kForward),
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
                      transporterMotor.set(Speeds.upperInDutyCycle);
                    },
                    // end
                    __ -> {
                      intakeMotor.set(0);
                      transporterMotor.set(0);
                      intakePiston.set(DoubleSolenoid.Value.kReverse);
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
                    __ -> {
                      intakeMotor.set(0);
                      intakePiston.set(DoubleSolenoid.Value.kReverse);
                    },
                    // isFinished
                    intakeBallTrigger.negate()),
                // decides by ball color
                this::isOurColor));
  }

  public Command manualCommand(
      BooleanSupplier intake,
      BooleanSupplier spit,
      BooleanSupplier transporter,
      BooleanSupplier spitTransporter,
      BooleanSupplier pistonState) {
    final double speed = 0.4;
    return new RunCommand(
        () -> {
          if (intake.getAsBoolean()) intakeMotor.set(-speed);
          else if (spit.getAsBoolean()) intakeMotor.set(speed);
          else intakeMotor.set(0);

          if (transporter.getAsBoolean()) transporterMotor.set(-speed);
          else if (spitTransporter.getAsBoolean()) transporterMotor.set(speed);
          else transporterMotor.set(0);

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
          transporterMotor.set(Speeds.upperShootDutyCycle);
          intakeMotor.set(Speeds.intakeShootDutyCycle);
        },
        // end
        _interrupted -> {
          transporterMotor.set(0);
          intakeMotor.set(0);
        },
        // isFinished
        // stop after we've shot a ball
        upperBallTrigger.negate());
  }

  private boolean isOurColor() {
    var colorVal = 0;
    boolean isOurColor = false;

    switch (DriverStation.getAlliance()) {
      case Red:
        colorVal = intakeSensor.getRed();
        isOurColor = colorVal < RED_THRESHOLD;
        break;
      case Blue:
        colorVal = intakeSensor.getBlue();
        isOurColor = colorVal < BLUE_THRESHOLD;
        break;
    }
    SmartDashboard.putNumber("color val: ", colorVal);
    return isOurColor;
  }

  @Override
  public void close() {
    this.intakeMotor.close();
    this.transporterMotor.close();
    this.upperSensor.close();
  }

  private enum Speeds {
    ;
    static final double ejectDutyCycle = 0.3;

    static final double intakeInDutyCycle = 0.4;
    static final double upperInDutyCycle = 0.4;

    static final double intakeShootDutyCycle = 0.2;
    static final double upperShootDutyCycle = 0.2;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Intake Cargo", intakeBallTrigger, null);
    builder.addBooleanProperty("Upper Cargo", upperBallTrigger, null);
    builder.addDoubleProperty("Cargo Count", ballCount::get, null);
  }

  public boolean isEmpty() {
    return ballCount.get() < 1;
  }

  @Override
  public void periodic() {
    if (intakePiston.get() != DoubleSolenoid.Value.kForward) {
      intakeMotor.set(0);
    }
    SmartDashboard.putBoolean("IsColor", isOurColor());
    SmartDashboard.putBoolean("isBall", upperBallTrigger.getAsBoolean());
    SmartDashboard.putNumber("sonic distance: ", upperSensor.getRangeMM());
  }
}
