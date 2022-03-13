package io.excaliburfrc.lib;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class ThreadedColorSensor {

  private static final double THRESHOLD = 0.5; // TODO: change that

  ColorSensorV3 sensor;
  private int proximity;
  private int red;
  private int green;
  private int blue;
  private int IR;
  private boolean connected;
  private Thread thread;

  public ThreadedColorSensor(I2C.Port port) {
    sensor = new ColorSensorV3(port);
    start();
  }

  public void start() {
    if (thread != null) {
      return;
    }
    thread = new Thread(this::updateValues);
    thread.start();
  }

  public void stop() {
    if (thread == null) {
      return;
    }
    thread.interrupt();
    thread = null;
  }

  private void updateValues() {
    while (!Thread.interrupted()) {
      double startTime = Timer.getFPGATimestamp();
      proximity = sensor.getProximity();
      red = sensor.getRed();
      green = sensor.getGreen();
      blue = sensor.getBlue();
      IR = sensor.getIR();
      connected = sensor.isConnected();
      double endTime = Timer.getFPGATimestamp();
      if (endTime - startTime > THRESHOLD) {
        DriverStation.reportError("it's take too much time to read from the sensor", false);
      }
      if (!isConnected()) {
        DriverStation.reportError("the sensor not connected", false);
      }
    }
    DriverStation.reportWarning("the thread that read from the sensor ended ended", false);
  }

  public int getProximity() {
    return proximity;
  }

  public int getRed() {
    return red;
  }

  public int getGreen() {
    return green;
  }

  public int getBlue() {
    return blue;
  }

  public int getIR() {
    return IR;
  }

  public boolean isConnected() {
    return connected;
  }
}
