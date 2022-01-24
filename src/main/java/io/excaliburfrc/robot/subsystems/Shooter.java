package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leader =
      new CANSparkMax(ShooterConstants.LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);

  public Shooter() {
    follower.follow(leader);
  }

  public Command activateCommand(DoubleSupplier speed) {
    return new RunCommand(() -> leader.set(speed.getAsDouble() * 0.5), this);
  }
}
