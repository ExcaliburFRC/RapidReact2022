package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 40; // PDP 4
    public static final int UPPER_MOTOR_ID = 41; // PDP 11
    public static final int PING = 9;
    public static final int ECHO = 8;
    public static final int COLOR_LIMIT = 60;
    public static final int SONIC_LIMIT = 69;
    public static final int MAX_BALLS = 2;
    public static final int FWD_CHANNEL = 3;
    public static final int REV_CHANNEL = 4;
  }

  public static class ClimberConstants {
    public static final int FORWARD_CHANNEL = 1;
    public static final int REVERSE_CHANNEL = 7;
    public static final int LEFT_MOTOR_ID = 30;
    public static final int RIGHT_MOTOR_ID = 31;

    public static final double OPEN_LOOP_CLIMB_DUTY_CYCLE = 0.9;

    public static final double SAFETY_DISTANCE = 0.1;
    public static final float FORWARD_SOFT_LIMIT = 174.78738f;
    public static final float REVERSE_SOFT_LIMIT = 0f;
    public static final double OPEN_HEIGHT = FORWARD_SOFT_LIMIT - SAFETY_DISTANCE;
    public static final double CLOSED_HEIGHT = REVERSE_SOFT_LIMIT;
    public static final double HALF_HEIGHT = OPEN_HEIGHT / 2.0;

    public static final Value ANGLED = Value.kReverse;
    public static final Value STRAIGHT = Value.kForward;
  }

  public static class ShooterConstants {
    public static final int LEADER_ID = 20;
    public static final int FOLLOWER_ID = 21;
    public static final int ENCODER_A = 0;
    public static final int ENCODER_B = 1;

    public static final double kS = 0.10898;
    public static final double kV = 0.065;
    //    public static final double kA = 0.031584; // we want 0 acceleration
    public static final double kP = 0.3;

    public static final double RATIO = 42.0 / 18.0;
    public static final double CPR = 1024;
    public static final double ROTATIONS_PER_PULSE = RATIO / CPR;

    // 121.45182291666667;
    public static final double FENDER_SHOT_RPM = 72;
    public static final double TOLERANCE = 2;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER_ID = 10;
    public static final int RIGHT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 12;
    public static final int RIGHT_FOLLOWER_ID = 13;

    public static final double kS = 0.11636;
    public static final double kV = 2.9229;
    public static final double kA = 0.14969;
    public static final double kP = 0.0013735;
    public static final double TRACKWIDTH_METERS = 0.69;
    public static final double GEARING = 1 / 10.71;
    public static final double METERS_PER_SHAFT_ROTATION = 0.4788;

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
  }

  public static class LedsConstants {
    public static final int LEDS_PORT = 0;
  }

  private Constants() {
    throw new UnsupportedOperationException("util class");
  }
}
