package io.excaliburfrc.lib.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CANEncoderSim {
  private final SimDouble position, velocity;

  public CANEncoderSim(boolean alt, int deviceID) {
    var device = new SimDeviceSim("SPARK MAX [" + deviceID + "]");
    position = device.getDouble(alt ? "Alt Encoder Position" : "Position");
    velocity = device.getDouble(alt ? "Alt Encoder Velocity" : "Velocity");
  }

  public void setVelocity(double vel) {
    velocity.set(vel);
  }

  public void setPosition(double pos) {
    position.set(pos);
  }
}
