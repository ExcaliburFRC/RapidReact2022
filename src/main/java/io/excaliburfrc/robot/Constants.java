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
    public static final int FWD_CHANNEL = 6;
    public static final int REV_CHANNEL = 7;
  }

  public static class ClimberConstants {
    public static final int FORWARD_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
    public static final int LEFT_MOTOR_ID = 51;
    public static final int RIGHT_MOTOR_ID = 52;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double MG = 1000;
    public static final double ANGLE = 45;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double MAX_VELOCITY = 0.5;
    public static final double MAX_ACCELERATION = 1;
    public static final double HEIGHT = 50;
    public static final double HEIGHT_TO_OPEN_PISTON = 20;
    public static final double SAFETY_DISTANCE = 0.1;
  }

  public static class ShooterConstants {
    public static final int LEADER_ID = 0;
    public static final int FOLLOWER_ID = 1;
    public static final int ENCODER_A = 2;
    public static final int ENCODER_B = 3;

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kS = 0;

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kV = 0;

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kP = 0;

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kI = 0;

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kD = 0;

    public static final double FENDER_SHOT_RPM = 2500;
    public static final double TOLERANCE = 10;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER_ID = 18;
    public static final int LEFT_FOLLOWER_ID = 17;
    public static final int RIGHT_LEADER_ID = 19;
    public static final int RIGHT_FOLLOWER_ID = 15;

    public static final double kS = 1;
    public static final double kV = 2;
    public static final double kA = 1;
    public static final double kP = 0;
    public static final double TRACKWIDTH_METERS = 0.69;
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
  }

  public static class LedsConstants {
    public static final int LEDS_PORT = 3;
  }
}
