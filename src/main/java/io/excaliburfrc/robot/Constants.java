// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package io.excaliburfrc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // theoretically this goes down to 1ms, but we don't want to clog anything
  public static final int minimal_FRAME_PERIOD = 5; // ms
  public static final int MAXIMAL_FRAME_PERIOD = 65535; // ms

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 22;
    public static final int UPPER_MOTOR_ID = 21;
    public static final int PING = 5;
    public static final int ECHO = 6;
    public static final int COLOR_LIMIT = 69;
    public static final int SONIC_LIMIT = 69;
    public static final int RED_THRESHOLD = 100;
    public static final int BLUE_THRESHOLD = 100;
    public static final int MAX_BALLS = 2;
    public static final int FWD_CHANNEL = 8;
    public static final int REV_CHANNEL = 9;
  }

  public static class ClimberConstants {
    public static final int FORWARD_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
    public static final int CLIMBER_SPARKMAX = 1;
    public static final int SENSOR_CHANNEL = 1;
  }

  public static class ShooterConstants {
    public static final int LEADER_ID = 0;
    public static final int FOLLOWER_ID = 1;
    public static final int ENCODER_A = 2;
    public static final int ENCODER_B = 3;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double FENDER_SHOT_RPM = 2500;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER_ID = 18;
    public static final int LEFT_FOLLOWER_ID = 17;
    public static final int RIGHT_LEADER_ID = 19;
    public static final int RIGHT_FOLLOWER_ID = 15;
    public static final int RIGHT_LINE_SENSOR_CHANNEL = 0;
    public static final int LEFT_LINE_SENSOR_CHANNEL = 1;
    public static final double BLACK_THRESHOLD = 10;
    public static final int LINE_SENSOR_SCALE = 100;
  }
}
