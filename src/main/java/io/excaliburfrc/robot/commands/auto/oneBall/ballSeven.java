package io.excaliburfrc.robot.commands.auto.oneBall;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import io.excaliburfrc.robot.commands.auto.Trajectories;
import io.excaliburfrc.robot.subsystems.Drive;
import io.excaliburfrc.robot.subsystems.LEDs;
import io.excaliburfrc.robot.subsystems.Superstructure;

import static io.excaliburfrc.robot.commands.auto.Trajectories.*;

public class ballSeven {


    public class ballFour extends SequentialCommandGroup {
        static final Pose2d start = new Pose2d(6.957, 4.349, Rotation2d.fromDegrees(330));
        static final Pose2d end = new Pose2d(7.83, 4.925, Rotation2d.fromDegrees(330));
        static final Translation2d ball7 = Trajectories.OUR_CARGO_7;
        static final Pose2d stop1 = new Pose2d(6.515, 4.564,Rotation2d.fromDegrees(180));

        public ballFour(Drive drive, LEDs leds, Superstructure superstructure) {
            super(
                    drive.resetOdometryCommand(start),
                    superstructure.shootBallsCommand(leds),
                    drive.followTrajectoryCommand(TrajectoryGenerator.generateTrajectory( start, innerWaypoints(stop1,ball7) , end , FORWARD)
                            ).alongWith(superstructure.intakeBallCommand())
                            .andThen(superstructure.shootBallsCommand(leds))
            );
        }
    }

}
//I first shoot two balls and then I go a little bit back to stop1 so I can take a turn
// and then go to ball 7 and the go back to shoot one more ball
//* off season ball
}
