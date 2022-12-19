package io.excaliburfrc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.excaliburfrc.robot.Constants;

public class SwerveModule{
    //create motors
    private final CANSparkMax _driveMotor;
    private final CANSparkMax _spinningMotor;

    private final RelativeEncoder _driveEncoder;
    private final RelativeEncoder _spinningEncoder;
    private final DutyCycleEncoder _absEncoder;
    private final boolean _absEncoderReversed;
    private final double _absEncoderOffsetRad;
    private final int _absEncoderChannel;

    private final PIDController _spinningPIDController;

    public SwerveModule(int driveMotorId,
                        int spinningMotorId,
                        boolean driveMotorReversed,
                        boolean spinningMotorReversed,
                        boolean absEncoderReversed,
                        int absEncoderChannel) {
        _absEncoderChannel = absEncoderChannel;
        _absEncoder = new DutyCycleEncoder(absEncoderChannel);
        _absEncoderReversed = absEncoderReversed;
        _absEncoderOffsetRad = _absEncoder.getPositionOffset() * 2 * Math.PI;//?


        _driveMotor = new CANSparkMax(driveMotorId, CANSparkMax.MotorType.kBrushless);
        _spinningMotor = new CANSparkMax(spinningMotorId, CANSparkMax.MotorType.kBrushless);

        _driveMotor.setInverted(driveMotorReversed);
        _spinningMotor.setInverted(spinningMotorReversed);

        _driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _spinningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        _driveEncoder = _driveMotor.getEncoder();
        _spinningEncoder = _spinningMotor.getEncoder();

        _driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meters);
        _driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        _spinningEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Rad);
        _spinningEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

        _spinningPIDController = new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
        _spinningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getDrivePosition() {
        return _driveEncoder.getPosition();
    }//return the position of the drive encoder

    public double getSpinningPosition() {
        return _spinningEncoder.getPosition();
    }//return

    public double getDriveVelocity() {
        return _driveEncoder.getVelocity();
    }

    public double getSpinningVelocity() {
        return _spinningEncoder.getVelocity();
    }

    public double getAbsEncoderRad() {
        double angle = _absEncoder.getAbsolutePosition();//?
        angle *= 2.0 * Math.PI;
        angle -= _absEncoderOffsetRad;
        return angle * (_absEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        _driveEncoder.setPosition(0);
        _spinningEncoder.setPosition(getAbsEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSpinningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        _driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.kPhysicalMaxSpeedMeterPerSec);
        _spinningMotor.set(_spinningPIDController.calculate(getSpinningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve [" + _absEncoderChannel + "] state ", state.toString());
    }

    public void stop() {//stop both motors
        _driveMotor.set(0);
        _spinningMotor.set(0);
    }
}