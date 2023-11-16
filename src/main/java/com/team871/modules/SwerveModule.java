package com.team871.modules;

import java.util.Map;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
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
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;
    private final CANCoder sensor;
    private final Translation2d leverArm;
    private final PIDController positionController;

    private GenericEntry directrionEntry;
    private GenericEntry speedEntry;
    private GenericEntry updateTime;
    private GenericEntry captureButton;
    private GenericEntry driveEnabled;
    private GenericEntry steerEnabled;
    private GenericEntry preOpt;

    public SwerveModule(
            final int driveId,
            final boolean driveInverted,
            final int steeringId,
            final boolean steerInverted,
            final int sensorId,
            final double sensorOffset,
            final boolean sensorInverted,
            final Translation2d leverArm,
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
        steeringMotor.setInverted(steerInverted);
        encoder = steeringMotor.getEncoder();
        encoder.setPositionConversionFactor(360.0d * Constants.STEERING_GEAR_RATION);
        encoder.setVelocityConversionFactor(360.0d * Constants.STEERING_GEAR_RATION / 60.0);
        steeringMotor.burnFlash();

        this.sensor = new CANCoder(sensorId);
        final CANCoderConfiguration sensorConfig = new CANCoderConfiguration();
        sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        sensorConfig.magnetOffsetDegrees = sensorOffset;
        sensorConfig.sensorDirection = sensorInverted;
        sensor.configAllSettings(sensorConfig);

        this.leverArm = leverArm;

        // TODO: These are BS PID numbers.
        this.positionController = new PIDController(.13, 0, 0);
        this.positionController.enableContinuousInput(0, 360);
        this.positionController.setTolerance(.5);

        targetLayout.add("PosPID", positionController)
                .withPosition(0, 0);

        targetLayout.addDouble("Angle", () -> sensor.getAbsolutePosition())
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Min", 0, "Max", 180))
                .withPosition(1, 0);

        this.directrionEntry = targetLayout.add("tDirection", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Min", 0, "Max", 180))
                .withPosition(2, 0)
                .getEntry();

        this.preOpt = targetLayout.add("preOpt", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Min", 0, "Max", 180))
                .withPosition(3, 0)
                .getEntry();

        this.speedEntry = targetLayout.add("tSpeedMPS", 0)
                .withPosition(0, 1)
                .getEntry();

        this.updateTime = targetLayout.add("tUpdate", 0)
                .withPosition(1, 1)
                .getEntry();

        targetLayout.addBoolean("AtSet", positionController::atSetpoint)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(2, 1);

        this.captureButton = targetLayout.add("CaptureOffset", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 2)
                .getEntry();

        this.driveEnabled = targetLayout.add("Drive Enabled", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(1, 2)
                .getEntry();

        this.steerEnabled = targetLayout.add("Steer Enabled", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(2, 2)
                .getEntry();

        // Initialize our current state to stopped and position 0
        setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public void setState(final SwerveModuleState newState) {
        final long tStart = RobotController.getFPGATime();
        if (RobotState.isTest()) {
            if (this.captureButton.getBoolean(false)) {
                // This is sad, but the sensor takes a few millis to actually apply the value.
                // if we read too quickly we will read the sensor plus the old offset.
                sensor.configMagnetOffset(0, 10);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                double newPosition = sensor.getAbsolutePosition();
                if (newPosition > 180) {
                    newPosition = 360 - newPosition;
                }

                sensor.configMagnetOffset(newPosition);
                captureButton.setBoolean(false);
                System.out.println("I set it to " + newPosition);
            }
        }

        final double currentDirection = sensor.getAbsolutePosition();
        final SwerveModuleState optimized = SwerveModuleState.optimize(newState,
                Rotation2d.fromDegrees(currentDirection));

        double optimizedAngle = optimized.angle.getDegrees();
        double degrees = Math.abs(optimized.speedMetersPerSecond) > 0
                ? optimizedAngle
                : positionController.getSetpoint();

        // Update controllers
        final double positionVoltage = positionController.calculate(currentDirection, degrees);
        steeringMotor.setVoltage(steerEnabled.getBoolean(true) ? positionVoltage : 0);

        driveMotor.set(driveEnabled.getBoolean(true) ? optimized.speedMetersPerSecond : 0);

        // Finally, update telemetry
        preOpt.setDouble(newState.angle.getDegrees());
        directrionEntry.setDouble(degrees);
        speedEntry.setDouble(optimized.speedMetersPerSecond);
        updateTime.setDouble((RobotController.getFPGATime() - tStart) / 1000.0d);
    }

    public Translation2d getLeverArm() {
        return leverArm;
    }
}
