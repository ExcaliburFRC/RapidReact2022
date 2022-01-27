package io.excaliburfrc.lib;

import static edu.wpi.first.wpilibj.DriverStation.reportError;

import com.revrobotics.REVLibError;

public final class CheckCAN {
  private CheckCAN() {
    throw new UnsupportedOperationException("this is a util class!!");
  }

  public static boolean ValidateREVCAN(REVLibError... statuses) {
    StringBuilder builder = new StringBuilder();
    int errors = 0;
    for (int i = 0; i < statuses.length; i++) {
      REVLibError err = statuses[i];
      if (err == REVLibError.kOk) continue;
      errors++;
      builder.append(String.format("\t[%d] %s", i, err));
    }
    if (errors == 0) return false;
    builder.insert(0, String.format("REV CAN Errors found: %d\n", errors));
    reportError(builder.toString(), true);
    return true;
  }
}
