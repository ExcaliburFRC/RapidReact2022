package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CAN.*;
import static io.excaliburfrc.robot.Constants.ClimberConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.ClimberConstants;
import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {
  private final DoubleSolenoid anglerPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimberConstants.FORWARD_CHANNEL,
          ClimberConstants.REVERSE_CHANNEL);

  private final ClimberSide left = new ClimberSide(LEFT_MOTOR_ID, true, 0.95);
  private final ClimberSide right = new ClimberSide(RIGHT_MOTOR_ID, false, 0.85);

  private static class ClimberSide {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final double Mspeed;

    public ClimberSide(int motorId, boolean isMotorReversed, double Mspeed) {
      motor = new CANSparkMax(motorId, MotorType.kBrushless);
      encoder = motor.getEncoder();
      this.Mspeed = Mspeed;

      ValidateREVCAN(
          // reset factory settings
          motor.restoreFactoryDefaults(),
          // set the motors to brake mode
          motor.setIdleMode(IdleMode.kBrake),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, StatusFramePeriods.DEFAULT),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, StatusFramePeriods.DO_NOT_SEND),
          motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, StatusFramePeriods.DEFAULT),
          motor.setSoftLimit(SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT),
          motor.enableSoftLimit(SoftLimitDirection.kReverse, true),
          motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.FORWARD_SOFT_LIMIT),
          motor.enableSoftLimit(SoftLimitDirection.kForward, true));
      motor.setInverted(isMotorReversed);

      encoder.setPosition(0);
    }

    public Command manualCommand(BooleanSupplier up, BooleanSupplier down) {
      return new FunctionalCommand(
          () -> {},
          () -> {
            if (up.getAsBoolean()) {
              motor.set(Mspeed / 4);
            } else if (down.getAsBoolean()) {
              motor.set(-Mspeed);
            } else {
              motor.set(0);
            }
          },
          __ -> motor.set(0),
          () -> false);
    }

    public Command tuneCommand(BooleanSupplier up, BooleanSupplier down) {
      return new FunctionalCommand(
          () -> {},
          () -> {
            if (up.getAsBoolean()) {
              motor.set(0.3);
            } else if (down.getAsBoolean()) {
              motor.set(-0.3);
            } else {
              motor.set(0);
            }
          },
          __ -> motor.set(0),
          () -> false);
    }

    public double getHeight() {
      return encoder.getPosition();
    }

    /** Disables soft limits for the duration of this command, and then resets the encoder. */
    public Command disableAndResetSoftLimits() {
      return new StartEndCommand(
          () -> {
            motor.enableSoftLimit(SoftLimitDirection.kForward, false);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
          },
          () -> {
            encoder.setPosition(0);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
          });
    }
  }

  public Command climberManualCommand(
      BooleanSupplier leftUp,
      BooleanSupplier leftDown,
      BooleanSupplier rightUp,
      BooleanSupplier rightDown,
      BooleanSupplier pistonAngled,
      BooleanSupplier pistonStraight) {
    return new ParallelCommandGroup(
        left.manualCommand(leftUp, leftDown),
        right.manualCommand(rightUp, rightDown),
        pistonCommand(pistonAngled, pistonStraight));
  }

  public Command climberTuneCommand(
      BooleanSupplier leftUp,
      BooleanSupplier leftDown,
      BooleanSupplier rightUp,
      BooleanSupplier rightDown,
      BooleanSupplier pistonAngled,
      BooleanSupplier pistonStraight) {
    return new ParallelCommandGroup(
        left.tuneCommand(leftUp, leftDown),
        right.tuneCommand(rightUp, rightDown),
        pistonCommand(pistonAngled, pistonStraight));
  }

  private FunctionalCommand pistonCommand(
      BooleanSupplier pistonAngled, BooleanSupplier pistonStraight) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          if (pistonAngled.getAsBoolean()) {
            anglerPiston.set(ANGLED);
          }
          if (pistonStraight.getAsBoolean()) {
            anglerPiston.set(STRAIGHT);
          }
        },
        __ -> {},
        () -> false);
  }

  public Command disableSoftLimits() {
    return left.disableAndResetSoftLimits().alongWith(right.disableAndResetSoftLimits());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    SendableRegistry.remove(anglerPiston);
    builder.addDoubleProperty("leftHeight", left::getHeight, null);
    builder.addDoubleProperty("rightHeight", right::getHeight, null);
  }
}
