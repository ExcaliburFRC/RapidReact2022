package io.excaliburfrc.lib;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class TriggerUtils {
  public static Trigger Falling(Trigger this$ref) {
    return new Trigger(
        new BooleanSupplier() {
          boolean previous;

          @Override
          public boolean getAsBoolean() {
            boolean present = this$ref.getAsBoolean();

            boolean ret;
            ret = previous && !present;
            previous = present;
            return ret;
          }
        });
  }
}
