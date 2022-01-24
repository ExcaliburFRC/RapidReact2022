package io.excaliburfrc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.INTAKE_MOTOR_ID);
  private final WPI_VictorSPX upperMotor = new WPI_VictorSPX(IntakeConstants.UPPER_MOTOR_ID);
  private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final Ultrasonic upperSensor = new Ultrasonic(IntakeConstants.PING, IntakeConstants.ECHO);
  private final DoubleSolenoid piston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, IntakeConstants.FWD_CHANNEL, IntakeConstants.REV_CHANNEL);

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
