package io.excaliburfrc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
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
  public static final class SwerveConstants {

    public static final double kTrackWidthMeters = 0.5842;//מעודכן
    public static final double kWheelBaseMeters = 0.5842;//מעודכן
    public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBaseMeters/2,-kTrackWidthMeters/2),
                    new Translation2d(kWheelBaseMeters/2,kTrackWidthMeters/2),
                    new Translation2d(-kWheelBaseMeters/2,-kTrackWidthMeters/2),
                    new Translation2d(-kWheelBaseMeters/2,kTrackWidthMeters/2));
    public static final double kTeleDriveMaxSpeedMetersPerSec = 0;//הגבלה תוכנתית
    public static final double kTeleDriveMaxAngularSpeedRadPerSec = 0;//הגבלה תוכנתית
    public static final double kMaxAccelerationUnitsPerSec = 0;//הגבלה תוכנתית
    public static final double kMaxAccelerationMetersPerSecSq = 0;//הגבלה תוכנתית
    public static final double kMaxAngularAccelerationReaPerSecSq = 0;//הגבלה תוכנתית
    public static final double kPhysicalMaxSpeedMeterPerSec = 0;//חישוב מכניקה + אלקטרוניקה
    public static final int kFrontLeftDriveMotorId = 0;//אלקטרוניקה סטאף
    public static final int kFrontRightDriveMotorId = 0;//אלקטרוניקה סטאף
    public static final int kBackLeftDriveMotorId = 0;//אלקטרוניקה סטאף
    public static final int kBackRightDriveMotorId = 0;//אלקטרוניקה סטאף


    public static final int kFrontLeftSpinningMotorId = 0;//אלקטרוניקה סטאף
    public static final int kFrontRightSpinningMotorId = 0;//אלקטרוניקה סטאף
    public static final int kBackLeftSpinningMotorId = 0;//אלקטרוניקה סטאף
    public static final int kBackRightSpinningMotorId = 0;//אלקטרוניקה סטאף


    public static final boolean kFrontLeftDriveEncoderReverse = false;//תוכנה סטאף
    public static final boolean kFrontRightDriveEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackLeftDriveEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackRightDriveEncoderReverse = false;//תוכנה סטאף

    public static final boolean kFrontLeftSpinningEncoderReverse = false;//תוכנה סטאף
    public static final boolean kFrontRightSpinningEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackLeftSpinningEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackRightSpinningEncoderReverse = false;//תוכנה סטאף

    public static final boolean kFrontLeftAbsEncoderReverse = false;//תוכנה סטאף
    public static final boolean kFrontRightAbsEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackLeftAbsEncoderReverse = false;//תוכנה סטאף
    public static final boolean kBackRightAbsEncoderReverse = false;//תוכנה סטאף

    public static final int kFrontLeftAbsEncoderChannel = 0;//אלקטרוניקה סטאף
    public static final int kFrontRightAbsEncoderChannel = 0;//אלקטרוניקה סטאף
    public static final int kBackLeftAbsEncoderChannel = 0;//אלקטרוניקה סטאף
    public static final int kBackRightAbsEncoderChannel = 0;//אלקטרוניקה סטאף

    public static final TrajectoryConfig kConfig = new TrajectoryConfig(
            kTeleDriveMaxSpeedMetersPerSec,
            kMaxAccelerationMetersPerSecSq).setKinematics(kSwerveKinematics);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
           new  TrapezoidProfile.Constraints(
                    kTeleDriveMaxAngularSpeedRadPerSec,
                    kMaxAngularAccelerationReaPerSecSq);
    public static final double kPXAuto = 0;
    public static final double kPYAuto = 0;
    public static final double kThetaAuto = 0;

    /*public static final PIDController xAutoController = new PIDController(0,0,0);//only kp needed
    public static final PIDController yAutoController = new PIDController(0,0,0);//only kp needed
    public static ProfiledPIDController thetaAutoController = new ProfiledPIDController(
            0,0,0,kThetaControllerConstraints);//only kp needed*/

  }

  public static final class ModuleConstants {
    //TODO update values
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio = 8.14;
    public static final double kTurningMotorGearRatio = 21.4285714;
    public static final double kDriveEncoderRot2Meters = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meters / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0;//תוכנה סטאף
  }
  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 40; // PDP 4
    public static final int UPPER_MOTOR_ID = 41; // PDP 11
    public static final int UPPER_PING = 9;
    public static final int UPPER_ECHO = 8;
    public static final int COLOR_LIMIT = 55;
    public static final int SONIC_LIMIT = 110;
    public static final int MAX_BALLS = 2;
    public static final int FWD_CHANNEL = 3;
    public static final int REV_CHANNEL = 4;
  }

  public static class ClimberConstants {
    public static final int FORWARD_CHANNEL = 1;
    public static final int REVERSE_CHANNEL = 7;
    public static final int LEFT_MOTOR_ID = 30;
    public static final int RIGHT_MOTOR_ID = 31;

    public static final float FORWARD_SOFT_LIMIT = 64.7f;
    public static final float REVERSE_SOFT_LIMIT = 2f;

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
    public static final double FENDER_SHOT_RPM = 68;
    public static final double TOLERANCE = 4;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_LEADER_ID = 10;
    public static final int RIGHT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 12;
    public static final int RIGHT_FOLLOWER_ID = 13;

    public static final double GEARING = 1.0 / 10.71;

    public static final double MOTOR_ROTATION_TO_METERS =
        GEARING * Math.PI * Units.inchesToMeters(6);

    public static final Translation2d HUB_POS = new Translation2d(8.247, 4.092);

    public static final double kS = 0.13086;
    public static final double kV = 2.8806;
    public static final double kA = 0.39188;
    public static final double kP = 0.045064;
    public static final DifferentialDriveKinematics driveKinematics =
        new DifferentialDriveKinematics(0.70);
    public static final double kP_ang = 0.006666666666666667;

    public static final Pose2d LOWER_FENDER =
          new Pose2d(7.797, 2.821, Rotation2d.fromDegrees(75));
  }

  public static class LedsConstants {
    public static final int LEDS_PORT = 0;
  }

  private Constants() {
    throw new UnsupportedOperationException("util class");
  }
}
