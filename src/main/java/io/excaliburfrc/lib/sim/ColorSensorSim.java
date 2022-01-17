package io.excaliburfrc.lib.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class ColorSensorSim {
  private final SimDouble m_red;
  private final SimDouble m_green;
  private final SimDouble m_blue;
  private final SimDouble m_ir;
  private final SimDouble m_proximity;

  public ColorSensorSim(I2C.Port port) {
    var device = new SimDeviceSim("REV Color Sensor V3", port.value, 0x52);
    m_red = device.getDouble("Red");
    m_green = device.getDouble("Green");
    m_blue = device.getDouble("Blue");
    m_ir = device.getDouble("IR");
    m_proximity = device.getDouble("Proximity");
  }

  public void setRed(int r) {
    m_red.set(r);
  }

  public void setGreen(int g) {
    m_green.set(g);
  }

  public void setBlue(int b) {
    m_blue.set(b);
  }

  public void setIR(int ir) {
    m_ir.set(ir);
  }

  public void setProximity(int prox) {
    m_proximity.set(prox);
  }
}
