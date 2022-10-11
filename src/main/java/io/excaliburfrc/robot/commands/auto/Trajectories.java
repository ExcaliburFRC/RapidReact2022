package io.excaliburfrc.robot.commands.auto;

import static edu.wpi.first.math.MathUtil.angleModulus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.List;

public enum Trajectories {
  ;
  // cargo indexes count clockwise from midline
  public static final Translation2d OUR_CARGO_1 = new Translation2d(7.49, 7.94);
  public static final Translation2d THEIR_CARGO_1 = new Translation2d(9, 8);
  public static final Translation2d THEIR_CARGO_2 = new Translation2d(11.55, 6.375);
  public static final Translation2d OUR_CARGO_2 = new Translation2d(12.17, 5);
  public static final Translation2d THEIR_CARGO_3 = new Translation2d(11.66, 2.03);
  public static final Translation2d OUR_CARGO_3 = new Translation2d(10.6, 0.93);
  public static final Translation2d THEIR_CARGO_4 = new Translation2d(9.23, 0.3);
  public static final Translation2d OUR_CARGO_4 = new Translation2d(7.684, 0.558);
  public static final Translation2d OUR_CARGO_5 = new Translation2d(5.06, 1.96);
  public static final Translation2d THEIR_CARGO_5 = new Translation2d(4.55, 3.25);
  public static final Translation2d OUR_CARGO_6 = new Translation2d(5, 6.15);
  public static final Translation2d THEIR_CARGO_6 = new Translation2d(5.10, 6.14);
  public static final Translation2d OUR_CARGO_7 = new Translation2d(3.18, 3.92);
  public static final Translation2d THEIR_CARGO_7 = new Translation2d(13.476, 4.17);

  public static final Translation2d OUR_TERMINAL_CARGO = new Translation2d(1.31, 1.5);
  public static final Translation2d THEIR_TERMINAL_CARGO = new Translation2d(15.519, 7.074);

  public static final Pose2d OUR_TOP_FENDER = new Pose2d( 6.94, 5.25, Rotation2d.fromDegrees(-47.65));

  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> innerWaypoints, Pose2d end, TrajectoryConfig config) {
    var traj = TrajectoryGenerator.generateTrajectory(start, innerWaypoints, end, config);
    return traj;
  }

  public static final TrajectoryConfig FORWARD = new TrajectoryConfig(1.8, 1.75).setReversed(false);
  public static final TrajectoryConfig REVERSE = new TrajectoryConfig(1.8, 1.75).setReversed(true);

  public static List<Translation2d> innerWaypoints(Translation2d... waypoints) {
    return List.of(waypoints);
  }

  public static Rotation2d radians(double rads) {
    return new Rotation2d(angleModulus(rads));
  }

  public static Rotation2d degrees(double deg) {
    return Rotation2d.fromDegrees(deg);
  }
}
