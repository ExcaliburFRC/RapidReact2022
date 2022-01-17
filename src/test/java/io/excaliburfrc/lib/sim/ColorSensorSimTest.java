package io.excaliburfrc.lib.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class ColorSensorSimTest {

  ColorSensorV3 real;
  ColorSensorSim sim;

  @BeforeEach
  void setUp() {
    real = new ColorSensorV3(I2C.Port.kMXP);
    sim = new ColorSensorSim(I2C.Port.kMXP);
  }

  @AfterEach
  void tearDown() {
    I2CJNI.i2CClose(I2C.Port.kMXP.value);
    real = null;
    sim = null;
    SimDeviceSim.resetData();
  }

  @ParameterizedTest
  @ValueSource(ints = {253, 0, 103, 33})
  void setRed(int red) {
    sim.setRed(red);
    assertEquals(red, real.getRed());
  }

  @ParameterizedTest
  @ValueSource(ints = {253, 0, 103, 33})
  void setBlue(int blue) {
    sim.setBlue(blue);
    assertEquals(blue, real.getBlue());
  }

  @ParameterizedTest
  @ValueSource(ints = {253, 0, 103, 33})
  void setGreen(int green) {
    sim.setGreen(green);
    assertEquals(green, real.getGreen());
  }

  @ParameterizedTest
  @ValueSource(ints = {253, 0, 103, 33})
  void setProximity(int proximity) {
    sim.setProximity(proximity);
    assertEquals(proximity, real.getProximity());
  }

  @ParameterizedTest
  @ValueSource(ints = {253, 0, 103, 33})
  void setIR(int ir) {
    sim.setIR(ir);
    assertEquals(ir, real.getIR());
  }
}
