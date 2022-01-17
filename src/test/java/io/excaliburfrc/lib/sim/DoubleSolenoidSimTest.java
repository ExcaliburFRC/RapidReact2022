package io.excaliburfrc.lib.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class DoubleSolenoidSimTest {
  DoubleSolenoid real;
  DoubleSolenoidSim sim;

  @BeforeAll
  static void initHAL() {
    assert HAL.initialize(500, 1);
  }

  @BeforeEach
  void setUp() {
    real = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    sim = new DoubleSolenoidSim(0, 1);
  }

  @AfterEach
  void tearDown() {
    real.close();
    real = null;
    sim = null;
  }

  @ParameterizedTest
  @EnumSource(DoubleSolenoid.Value.class)
  void test(DoubleSolenoid.Value value) {
    real.set(value);
    assertEquals(value, real.get());
    assertEquals(value, sim.get());
  }
}
