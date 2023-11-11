package com.team871.modules;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SparkMaxSendableWrapper implements Sendable {
    final CANSparkMax mc;

    public SparkMaxSendableWrapper(CANSparkMax mc) {
        this.mc = mc;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
		builder.setActuator(true);
		builder.setSafeState(mc::stopMotor);
		builder.addDoubleProperty("Value", mc::get, mc::set);
        builder.addDoubleProperty("Position", () -> mc.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Velocity", () -> mc.getEncoder().getVelocity(), null);
    }
}
