package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);
  private final CANSparkMax leftMotor =
      new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkMaxPIDController leftController = leftMotor.getPIDController();
  private final SparkMaxPIDController rightController = rightMotor.getPIDController();

  private final ElevatorFeedforward upFF = new ElevatorFeedforward(kS, MG, kV, kA);
  private final TrapezoidProfile.Constraints upConstraints =
      new TrapezoidProfile.Constraints(
          upFF.maxAchievableVelocity(12, 1), upFF.maxAchievableAcceleration(12, 1));

  private final ElevatorFeedforward diagonalFF =
      new ElevatorFeedforward(kS, MG * Math.cos(ANGLE), kV, kA);
  private final TrapezoidProfile.Constraints diagonalConstraints =
      new TrapezoidProfile.Constraints(
          diagonalFF.maxAchievableVelocity(12, 1), diagonalFF.maxAchievableAcceleration(12, 1));

  private final TrapezoidProfile.State initState = new TrapezoidProfile.State(0, 0);

  private final TrapezoidProfile fullUpProfile =
      new TrapezoidProfile(upConstraints, new TrapezoidProfile.State(FULL_UP_HEIGHT, 0), initState);
  private final TrapezoidProfile someUpProfile =
      new TrapezoidProfile(upConstraints, new TrapezoidProfile.State(SOME_UP_HEIGHT, 0), initState);
  private final TrapezoidProfile diagonalProfile =
      new TrapezoidProfile(
          diagonalConstraints,
          new TrapezoidProfile.State(FULL_UP_HEIGHT, 0),
          new TrapezoidProfile.State(SOME_UP_HEIGHT, 0));

  public Climber() {
    ValidateREVCAN(
        // reset factory settings
        leftMotor.restoreFactoryDefaults(),
        rightMotor.restoreFactoryDefaults(),
        // set the motors to brake mode
        leftMotor.setIdleMode(IdleMode.kBrake),
        rightMotor.setIdleMode(IdleMode.kBrake),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
        // set up PID parameters
        leftController.setFeedbackDevice(leftEncoder),
        leftController.setP(kP),
        leftController.setI(kI),
        leftController.setD(kD),
        rightController.setFeedbackDevice(rightEncoder),
        rightController.setP(kP),
        rightController.setI(kI),
        rightController.setD(kD));
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
    anglerPiston.close();
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

  public void activateMotors(MotorMode m) {
    leftMotor.set(m.dutyCycle);
    rightMotor.set(m.dutyCycle);
  }

  public void openAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kForward);
  }

  public void closeAngler() {
    anglerPiston.set(DoubleSolenoid.Value.kReverse);
  }

  private Command reachSideHeight(
      SparkMaxPIDController controller, ElevatorFeedforward feedforward, TrapezoidProfile profile) {
    return new TrapezoidProfileCommand(
        profile,
        setpoint ->
            controller.setReference(
                setpoint.velocity,
                ControlType.kVelocity,
                0,
                feedforward.calculate(setpoint.velocity),
                ArbFFUnits.kVoltage),
        this);
  }

  private Command reachBothHeight(ElevatorFeedforward feedforward, TrapezoidProfile profile) {
    return reachSideHeight(leftController, feedforward, profile)
        .alongWith(reachSideHeight(rightController, feedforward, profile));
  }

  public Command fullUpCommand() {
    return reachBothHeight(upFF, fullUpProfile);
  }

  public Command someUpCommand() {
    return reachBothHeight(upFF, someUpProfile);
  }

  public Command diagonalCommand() {
    return reachBothHeight(diagonalFF, diagonalProfile);
  }

  public Command offCommand() {
    return new InstantCommand(() -> activateMotors(MotorMode.OFF));
  }

  public Command openAnglerCommand() {
    return new InstantCommand(this::openAngler, this);
  }

  public Command closeAnglerCommand() {
    return new InstantCommand(this::closeAngler, this);
  }

  public Command climberManualCommand(DoubleSupplier motorSpeed, BooleanSupplier piston) {
    return new RunCommand(
        () -> {
          leftMotor.set(motorSpeed.getAsDouble());
          rightMotor.set(motorSpeed.getAsDouble());
          if (piston.getAsBoolean()) this.anglerPiston.toggle();
        },
        this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("leftHeight", leftEncoder::getPosition, null);
    builder.addDoubleProperty("rightHeight", rightEncoder::getPosition, null);
  }
}
