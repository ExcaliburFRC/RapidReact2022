package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.SENSOR_CHANNEL;
import static org.junit.Assert.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.CTREPCMSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.Before;
import org.junit.Test;

public class ClimberTest {
  private Climber climber;

  private DIOSim sensorSim;
  private CTREPCMSim pcmSim;
  private SimDeviceSim motorSim;

  @Before
  public void setup() {
    assert HAL.initialize(500, 1);
    climber = new Climber();
    sensorSim = new DIOSim(SENSOR_CHANNEL);
    DriverStationSim.setEnabled(true);
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
}
