package io.excaliburfrc.robot.commands.auto.twoBalls;

import static io.excaliburfrc.robot.commands.auto.Trajectories.OUR_CARGO_4;
import static io.excaliburfrc.robot.commands.auto.Trajectories.OUR_CARGO_5;
import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class fiveFour extends SequentialCommandGroup {
	static final Pose2d START = new Pose2d(8.2, 2.40, fromDegrees(90));
	static final Pose2d BALL_1 = new Pose2d(OUR_CARGO_5.getX(), OUR_CARGO_5.getY(), fromDegrees(200));
	static final Pose2d BALL_2 = new Pose2d(OUR_CARGO_4.getX(), OUR_CARGO_5.getY(), fromDegrees(0));
	static final Pose2d STOP_1 = new Pose2d(8.168, 1.676, START.getRotation());

	public fiveFour(Drive drive, LEDs leds, Superstructure superstructure) {
		super(
				drive.resetOdometryCommand(START),
				superstructure.shootBallsCommand(leds),
				drive.followTrajectoryCommand(START, innerWaypoints(), STOP_1, REVERSE()),
				new SequentialCommandGroup(
                        drive.followTrajectoryCommand(STOP_1, innerWaypoints(), BALL_1, FORWARD()),
                        drive.followTrajectoryCommand(BALL_1, innerWaypoints(), BALL_2, FORWARD())
                ).alongWith(superstructure.intakeBallCommand(), superstructure.intakeBallCommand()),
				drive.followTrajectoryCommand(BALL_2, innerWaypoints(), START, FORWARD()),
				superstructure.shootBallsCommand(leds));
	}
}
