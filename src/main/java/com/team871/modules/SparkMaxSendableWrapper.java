package com.team871.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxSendableWrapper implements Sendable {
    final CANSparkMax mc;

    public SparkMaxSendableWrapper(CANSparkMax mc) {
        this.mc = mc;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(mc::stopMotor);
        builder.addDoubleProperty("Output", mc::get, mc::set);
        builder.addDoubleProperty("Position", () -> mc.getEncoder().getPosition(), p -> mc.getEncoder().setPosition(p));
        builder.addDoubleProperty("Velocity", () -> mc.getEncoder().getVelocity(), null);
        builder.addBooleanProperty("BreakMode", () -> mc.getIdleMode() == IdleMode.kBrake,
                (brake) -> mc.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast));
        builder.addDoubleProperty("Temp", () -> mc.getMotorTemperature(), null);
    }
}
