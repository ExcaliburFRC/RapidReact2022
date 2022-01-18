package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.ClimberConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.excaliburfrc.lib.sim.DoubleSolenoidSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimberTest {
  private Climber climber;

  private DIOSim sensorSim;
  private DoubleSolenoidSim anglerSim;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 1);
    climber = new Climber();
    anglerSim = new DoubleSolenoidSim(FORWARD_CHANNEL, REVERSE_CHANNEL);
    sensorSim = new DIOSim(SENSOR_CHANNEL);
    DriverStationSim.setEnabled(true);
  }

  @AfterEach
  public void close() {
    climber.close();
    climber = null;
    sensorSim.resetData();
    sensorSim = null;
    anglerSim.resetData();
    anglerSim = null;
  }

  @Test
  public void downStopsOnSensor() {
    final var testCommand = climber.downCommand();
    sensorSim.setValue(false);

    testCommand.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(Climber.MotorMode.DOWN.dutyCycle, climber._getSpeed(), 1e-6);

    sensorSim.setValue(true);
    CommandScheduler.getInstance().run();
    assertEquals(Climber.MotorMode.OFF.dutyCycle, climber._getSpeed(), 1e-6);
  }

  @Test
  public void upStopsOnSensor() {
    final var testCommand = climber.upCommand();
    sensorSim.setValue(false);

    testCommand.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(Climber.MotorMode.UP.dutyCycle, climber._getSpeed(), 1e-6);

    sensorSim.setValue(true);
    CommandScheduler.getInstance().run();
    assertEquals(Climber.MotorMode.OFF.dutyCycle, climber._getSpeed(), 1e-6);
  }

  @Test
  public void anglerOpens() {
    final var testCommand = climber.openAnglerCommand();
    assertEquals(DoubleSolenoid.Value.kOff, anglerSim.get());

    testCommand.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(DoubleSolenoid.Value.kForward, anglerSim.get());
  }

  @Test
  public void anglerCloses() {
    final var testCommand = climber.closeAnglerCommand();
    assertEquals(DoubleSolenoid.Value.kOff, anglerSim.get());

    testCommand.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(DoubleSolenoid.Value.kReverse, anglerSim.get());
  }
}
