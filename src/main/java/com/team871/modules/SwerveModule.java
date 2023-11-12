package com.team871.modules;

import java.util.Map;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team871.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModule {
    public enum ModulePosition {
        FrontLeft,
        FrontRight,
        RearLeft,
        RearRight
    }

    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;
    private final CANCoder sensor;
    private final Translation2d leverArm;
    private final PIDController positionController;

    private GenericEntry directrionEntry;
    private GenericEntry speedEntry;
    private GenericEntry updateTime;
    private GenericEntry captureButton;

    public SwerveModule(final ModulePosition modPos,
            final int driveId,
            final int steeringId,
            final int sensorId,
            final ShuffleboardLayout targetLayout) {

        this.driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        RelativeEncoder encoder = driveMotor.getEncoder();
        final double positionFactor = Constants.WHEEL_LENGTH_METERS * Constants.DRIVE_GEAR_RATIO;

        encoder.setPositionConversionFactor(positionFactor);
        encoder.setVelocityConversionFactor(positionFactor / 60.0);
        driveMotor.enableVoltageCompensation(12);
        driveMotor.burnFlash();

        steeringMotor = new CANSparkMax(steeringId, MotorType.kBrushless);
        steeringMotor.enableVoltageCompensation(12);
        encoder = steeringMotor.getEncoder();
        encoder.setPositionConversionFactor(360.0d * Constants.STEERING_GEAR_RATION);
        encoder.setVelocityConversionFactor(360.0d * Constants.STEERING_GEAR_RATION / 60.0);
        steeringMotor.burnFlash();

        this.sensor = new CANCoder(sensorId);
        final CANCoderConfiguration sensorConfig = new CANCoderConfiguration();
        sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        sensorConfig.magnetOffsetDegrees = Constants.getMagnetOffset(modPos);
        sensorConfig.sensorDirection = Constants.getSensorDirection(modPos);
        sensor.configAllSettings(sensorConfig);

        this.leverArm = Constants.getLeverArm(modPos);

        // TODO: These are BS PID numbers.
        this.positionController = new PIDController(1 / 180.0d, 0, 1 / 360.0d);

        targetLayout.add("Steering", new SparkMaxSendableWrapper(steeringMotor))
                .withWidget(BuiltInWidgets.kMotorController);
        targetLayout.add("Drive", new SparkMaxSendableWrapper(driveMotor))
                .withWidget(BuiltInWidgets.kMotorController);
        targetLayout.add("PosPID", positionController);
        targetLayout.add("Angle", new CANCoderSendableWrapper(sensor));
        this.directrionEntry = targetLayout.add("tDirection", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Min", "0", "Max", "360"))
                .getEntry();
        this.speedEntry = targetLayout.add("tSpeedMPS", 0).getEntry();
        this.updateTime = targetLayout.add("tUpdate", 0).getEntry();
        this.captureButton = targetLayout.add("CaptureOffset", false)
                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

        // Initialize our current state to stopped and position 0
        setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public void setState(final SwerveModuleState newState) {
        if (RobotState.isTest()) {
            if (this.captureButton.getBoolean(false)) {
                final double readPosition = sensor.getAbsolutePosition();
                final double currentOffset = sensor.configGetMagnetOffset();
                sensor.configMagnetOffset(readPosition - currentOffset);
                captureButton.setBoolean(false);
                System.out.println("I set it to " + (readPosition - currentOffset));
            }
        }

        final long tStart = RobotController.getFPGATime();
        final double degrees = newState.angle.getDegrees();

        // Update controllers
        final double positionVoltage = positionController.calculate(sensor.getAbsolutePosition(), degrees);
        steeringMotor.setVoltage(positionVoltage);
        driveMotor.getPIDController().setReference(newState.speedMetersPerSecond, ControlType.kVelocity);

        // Finally, update telemetry
        directrionEntry.setDouble(degrees);
        speedEntry.setDouble(newState.speedMetersPerSecond);
        updateTime.setDouble((RobotController.getFPGATime() - tStart) / 1000.0d);
    }

    public Translation2d getLeverArm() {
        return leverArm;
    }
}
