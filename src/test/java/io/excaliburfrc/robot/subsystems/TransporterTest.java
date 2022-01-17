package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.TranporterConstants.COLOR_LIMIT;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.excaliburfrc.lib.sim.ColorSensorSim;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TransporterTest {
  Transporter transporter;

  ColorSensorSim sensorSim;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 1);

    transporter = new Transporter();
    transporter.init();

    sensorSim = new ColorSensorSim(I2C.Port.kMXP);

    DriverStationSim.setEnabled(true);
  }

  @Test
  public void ensureCommandScheduling() {
    sensorSim.setProximity(COLOR_LIMIT - 50);
    AtomicInteger scheduledCounter = new AtomicInteger(0);
    CommandScheduler.getInstance().onCommandInitialize(cmd -> scheduledCounter.incrementAndGet());
    CommandScheduler.getInstance().run();
    assertEquals(0, scheduledCounter.get());

    sensorSim.setProximity(COLOR_LIMIT + 50);
    CommandScheduler.getInstance().run();
    assertEquals(1, scheduledCounter.get());
  }

  @AfterEach
  public void teardown() {}
}
