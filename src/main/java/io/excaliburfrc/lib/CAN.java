package io.excaliburfrc.lib;

import static edu.wpi.first.wpilibj.DriverStation.reportError;

import com.revrobotics.REVLibError;

public final class CAN {
  public enum StatusFramePeriods {
    ;
    public static final int HIGH_PRIORITY = 5; // ms
    public static final int DEFAULT = 20; // ms
    public static final int DO_NOT_SEND = 65535; // ms
  }
  // theoretically this goes down to 1ms, but we don't want to clog anything

  public static void ValidateREVCAN(REVLibError... statuses) {
    StringBuilder builder = new StringBuilder();
    int errors = 0;
    for (int i = 0; i < statuses.length; i++) {
      REVLibError err = statuses[i];
      if (err == REVLibError.kOk) continue;
      errors++;
      builder.append(String.format("\t[%d] %s", i, err));
    }
    if (errors == 0) return;
    builder.insert(0, String.format("REV CAN Errors found: %d\n", errors));
    reportError(builder.toString(), true);
  }

  private CAN() {
    throw new UnsupportedOperationException("this is a util class!!");
  }
}
