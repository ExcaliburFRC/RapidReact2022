package io.excaliburfrc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.lib.RunEndCommand;
import io.excaliburfrc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
    private final SwerveModule _frontLeft = new SwerveModule(
            Constants.SwerveConstants.kFrontLeftDriveMotorId,
            Constants.SwerveConstants.kFrontLeftSpinningMotorId,
            Constants.SwerveConstants.kFrontLeftDriveEncoderReverse,
            Constants.SwerveConstants.kFrontLeftSpinningEncoderReverse,
            Constants.SwerveConstants.kFrontLeftAbsEncoderReverse,
            Constants.SwerveConstants.kFrontLeftAbsEncoderChannel);

    private final SwerveModule _frontRight = new SwerveModule(
            Constants.SwerveConstants.kFrontRightDriveMotorId,
            Constants.SwerveConstants.kFrontRightSpinningMotorId,
            Constants.SwerveConstants.kFrontRightDriveEncoderReverse,
            Constants.SwerveConstants.kFrontRightSpinningEncoderReverse,
            Constants.SwerveConstants.kFrontRightAbsEncoderReverse,
            Constants.SwerveConstants.kFrontRightAbsEncoderChannel);

    private final SwerveModule _backLeft = new SwerveModule(
            Constants.SwerveConstants.kBackLeftDriveMotorId,
            Constants.SwerveConstants.kBackLeftSpinningMotorId,
            Constants.SwerveConstants.kBackLeftDriveEncoderReverse,
            Constants.SwerveConstants.kBackLeftSpinningEncoderReverse,
            Constants.SwerveConstants.kBackLeftAbsEncoderReverse,
            Constants.SwerveConstants.kBackLeftAbsEncoderChannel);

    private final SwerveModule _backRight = new SwerveModule(
            Constants.SwerveConstants.kBackRightDriveMotorId,
            Constants.SwerveConstants.kBackRightSpinningMotorId,
            Constants.SwerveConstants.kBackRightDriveEncoderReverse,
            Constants.SwerveConstants.kBackRightSpinningEncoderReverse,
            Constants.SwerveConstants.kBackRightAbsEncoderReverse,
            Constants.SwerveConstants.kBackRightAbsEncoderChannel);

    private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
            Constants.SwerveConstants.kSwerveKinematics,
            getRotation2d());

    public Command resetOdometryCommand(Pose2d pose) {
        return new InstantCommand(() -> _odometry.resetPosition(pose, getRotation2d()));
    }

    public Pose2d getOdomertyPose() {
        return _odometry.getPoseMeters();
    }

    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyro();
            } catch (Exception e) {
            }
        }).start();
    }

    public void resetGyro() {
        _gyro.reset();
    }

    public double getDegrees() {
        return Math.IEEEremainder(_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    @Override
    public void periodic() {
        _odometry.update(
                getRotation2d(),
                _frontLeft.getState(),
                _frontRight.getState(),
                _backLeft.getState(),
                _backRight.getState());
        SmartDashboard.putNumber("Robot heading to: ", getDegrees());
        SmartDashboard.putString("Robot location: ",getOdomertyPose().getTranslation().toString());
    }

    public void stopModules() {
        _frontLeft.stop();
        _frontRight.stop();
        _backLeft.stop();
        _backRight.stop();
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Constants.SwerveConstants.kPhysicalMaxSpeedMeterPerSec);

        _frontLeft.setDesiredState(states[0]);
        _frontRight.setDesiredState(states[1]);
        _backLeft.setDesiredState(states[2]);
        _backRight.setDesiredState(states[3]);
    }

    public Command driveSwerveCommand(DoubleSupplier xSpeedSupplier,
                                      DoubleSupplier ySpeedSupplier,
                                      DoubleSupplier spinningSpeedSupplier,
                                      BooleanSupplier fieldOriented) {
        double joystickDeadBand = 0.2;
        SlewRateLimiter
                xLimiter = new SlewRateLimiter(Constants.SwerveConstants.kMaxAccelerationUnitsPerSec),
                yLimiter = new SlewRateLimiter(Constants.SwerveConstants.kMaxAccelerationUnitsPerSec),
                spinningLimiter = new SlewRateLimiter(Constants.SwerveConstants.kMaxAccelerationUnitsPerSec);
        return new RunEndCommand(
                () -> {

                    double xSpeed = xSpeedSupplier.getAsDouble(),
                            ySpeed = ySpeedSupplier.getAsDouble(),
                            spinningSpeed = spinningSpeedSupplier.getAsDouble();

                    xSpeed = Math.abs(xSpeed) < joystickDeadBand ? 0 : xSpeed;
                    ySpeed = Math.abs(ySpeed) < joystickDeadBand ? 0 : ySpeed;
                    spinningSpeed = Math.abs(spinningSpeed) < joystickDeadBand ? 0 : spinningSpeed;

                    xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveConstants.kTeleDriveMaxSpeedMetersPerSec;
                    ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveConstants.kTeleDriveMaxSpeedMetersPerSec;
                    spinningSpeed = spinningLimiter.calculate(spinningSpeed)
                            * Constants.SwerveConstants.kTeleDriveMaxAngularSpeedRadPerSec;

                    ChassisSpeeds chassisSpeeds = fieldOriented.getAsBoolean() ?
                            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, spinningSpeed, getRotation2d()) :
                            new ChassisSpeeds(xSpeed, ySpeed, spinningSpeed);

                    SwerveModuleState moduleStates[] =
                            Constants.SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
                    setModulesStates(moduleStates);
                },
                () -> stopModules());
    }
}