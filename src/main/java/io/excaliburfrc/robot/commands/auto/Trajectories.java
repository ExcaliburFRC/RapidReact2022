package io.excaliburfrc.robot.commands.auto;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static io.excaliburfrc.robot.Constants.DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
import static io.excaliburfrc.robot.Constants.DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public enum Trajectories {
  ;
  public static final Translation2d ORIGIN = new Translation2d(0, 0);

  public static final Pose2d OUR_TOP_FENDER = new Pose2d(7.5, 4.45, degrees(159.0));
  public static final Pose2d OUR_BOTTOM_FENDER = new Pose2d(8, 3.3, degrees(204.0));
  public static final Pose2d THEIR_TOP_FENDER = new Pose2d(8.7, 5.0, degrees(114.0));
  public static final Pose2d THEIR_BOTTOM_FENDER = new Pose2d(9.2, 3.8, degrees(69.0));

  // cargo indexes count clockwise from midline
  public static final Translation2d OUR_CARGO_1 = new Translation2d(7.49, 7.94);
  public static final Translation2d THEIR_CARGO_1 = new Translation2d(9, 8);
  public static final Translation2d THEIR_CARGO_2 = new Translation2d(11.55, 6.375);
  public static final Translation2d OUR_CARGO_2 = new Translation2d(12.17, 5);
  public static final Translation2d THEIR_CARGO_3 = new Translation2d(11.66, 2.03);
  public static final Translation2d OUR_CARGO_3 = new Translation2d(10.6, 0.93);
  public static final Translation2d THEIR_CARGO_4 = new Translation2d(9.23, 0.3);
  public static final Translation2d OUR_CARGO_4 = new Translation2d(7.7, 0.3);
  public static final Translation2d OUR_CARGO_5 = new Translation2d(5.18, 1.9);
  public static final Translation2d THEIR_CARGO_5 = new Translation2d(4.55, 3.25);
  public static final Translation2d OUR_CARGO_6 = new Translation2d(15.55, 7.12);
  public static final Translation2d THEIR_CARGO_6 = new Translation2d(6.1, 7.3);

  public static final Translation2d OUR_TERMINAL_CARGO = new Translation2d(1.15, 1.115);
  public static final Translation2d THEIR_TERMINAL_CARGO = new Translation2d(1.15, 1.115);

  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> innerWaypoints, Pose2d end, TrajectoryConfig config) {
    var traj = TrajectoryGenerator.generateTrajectory(start, innerWaypoints, end, config);
    var name = "trajectory " + Thread.currentThread().getStackTrace()[2].getFileName();
    return traj;
  }

  public static TrajectoryConfig FORWARD() {
    return new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setReversed(false);
  }

  public static TrajectoryConfig REVERSE() {
    return new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setReversed(true);
  }

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
