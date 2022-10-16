package io.excaliburfrc.robot.commands.auto.twoBalls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class TwoBalls_SevenFive extends SequentialCommandGroup {
    static final Pose2d BALL1 = new Pose2d(3.379, 4.122, Rotation2d.fromDegrees(180));
    static final Pose2d START = new Pose2d(6.925, 4.327, Rotation2d.fromDegrees(-20));
    static final Pose2d STOP1 = new Pose2d(6.925, 4.327, Rotation2d.fromDegrees(-20));
    static final Pose2d END = new Pose2d(7.453, 3.120, Rotation2d.fromDegrees(80));
    static final Pose2d BALL2 = new Pose2d(5.071, 1.859, Rotation2d.fromDegrees(-100));

    public TwoBalls_SevenFive(Drive drive, LEDs leds, Superstructure superstructure) {
        super(
                drive.resetOdometryCommand(START),
                superstructure.shootBallsCommand(leds),
                new SequentialCommandGroup(
                drive.followTrajectoryCommand(START, innerWaypoints(), STOP1, REVERSE),
                drive.followTrajectoryCommand(STOP1, innerWaypoints(), BALL1, FORWARD),
                drive.followTrajectoryCommand(BALL1, innerWaypoints(), BALL2, FORWARD),
                drive.followTrajectoryCommand(BALL2, innerWaypoints(), END, FORWARD)
                ).alongWith(superstructure.intakeBallCommand()),
                superstructure.shootBallsCommand(leds));
    }
}
