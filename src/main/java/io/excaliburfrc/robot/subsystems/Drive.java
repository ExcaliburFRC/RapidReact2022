package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;

import static io.excaliburfrc.lib.CheckCAN.ValidateREVCAN;
import static io.excaliburfrc.robot.Constants.MAXIMAL_FRAME_PERIOD;
import static io.excaliburfrc.robot.Constants.minimal_FRAME_PERIOD;

public class Drive extends SubsystemBase {
  private final CANSparkMax leftLeader =
      new CANSparkMax(DrivetrainConstants.LEFT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(DrivetrainConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(DrivetrainConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(DrivetrainConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  public Drive() {
    ValidateREVCAN(
          // reset factory settings
          leftLeader.restoreFactoryDefaults(),
          leftFollower.restoreFactoryDefaults(),
          rightLeader.restoreFactoryDefaults(),
          rightFollower.restoreFactoryDefaults(),
          // set the motors to coast mode -- we don't want to break them!
          leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake),
          leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake),
          rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake),
          rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake),
          // have the leader send its applied output as frequently as possible,
          // to speed up follower response
          leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, minimal_FRAME_PERIOD),
          leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, minimal_FRAME_PERIOD),
          leftLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
          rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, minimal_FRAME_PERIOD),
          rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, minimal_FRAME_PERIOD),
          rightLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, minimal_FRAME_PERIOD),
          // other status frames can be reduced to almost never
          leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
          leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
          leftFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
          rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXIMAL_FRAME_PERIOD),
          rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAXIMAL_FRAME_PERIOD),
          rightFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAXIMAL_FRAME_PERIOD),
          // setup following
          leftFollower.follow(leftLeader),
          rightFollower.follow(rightLeader));
  }

  public Command arcadeDriveCommend(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return new RunCommand(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), this);
  }
}
