package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.*;
import io.excaliburfrc.robot.Constants.IntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax upperMotor =
      new CANSparkMax(IntakeConstants.UPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Ultrasonic upperSensor = new Ultrasonic(IntakeConstants.PING, IntakeConstants.ECHO);
  private final DoubleSolenoid piston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, IntakeConstants.FWD_CHANNEL, IntakeConstants.REV_CHANNEL);

  public Intake() {
    ValidateREVCAN(
        // reset factory settings
        intakeMotor.restoreFactoryDefaults(),
        upperMotor.restoreFactoryDefaults(),
        // set the motors to coast mode -- we don't want to break them!
        upperMotor.setIdleMode(IdleMode.kBrake),
        intakeMotor.setIdleMode(IdleMode.kBrake),
        // have the upperMotor send its applied output as frequently as possible,
        // to speed up intakeMotor response
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        upperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD));
  }

  public Command manualCommand(
      DoubleSupplier intake, DoubleSupplier upper, BooleanSupplier pistonState) {
    return new RunCommand(
        () -> {
          intakeMotor.set(intake.getAsDouble());
          upperMotor.set(upper.getAsDouble());
          if (pistonState.getAsBoolean()) piston.toggle();
        },
        this);
  }
}
