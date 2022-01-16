package frc.robot.subsystems;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import static frc.robot.Constants.TranporterConstants.COLOR_LIMIT;
import static org.junit.Assert.*;

public class TransporterTest {
    Transporter transporter;
    SimDouble proximity;
    SimDouble red;
    SimDouble blue;

    @Before
    public void setup() {
        assert HAL.initialize(500, 1);

        transporter = new Transporter();
        transporter.init();

        var device = new SimDeviceSim("REV Color Sensor V3", I2C.Port.kMXP.value, 0x52);
        proximity = device.getDouble("Proximity");
        red = device.getDouble("Red");
        blue = device.getDouble("Blue");
        DriverStationSim.setEnabled(true);
    }

    @Test
    public void ensureCommandScheduling() {
        proximity.set(COLOR_LIMIT - 50);
        AtomicInteger scheduledCounter = new AtomicInteger(0);
        CommandScheduler.getInstance().onCommandInitialize(cmd -> scheduledCounter.incrementAndGet());
        CommandScheduler.getInstance().run();
        assertEquals(0, scheduledCounter.get());

        proximity.set(COLOR_LIMIT + 50);
        CommandScheduler.getInstance().run();
        assertEquals(1, scheduledCounter.get());
    }

    @Parameter


    @After
    public void teardown() {
    }
}