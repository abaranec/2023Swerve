package com.team871.modules;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CANCoderSendableWrapper implements Sendable {
    private final CANCoder cc;

    public CANCoderSendableWrapper(CANCoder cc) {
        this.cc = cc;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("AbsPosition", () -> cc.getAbsolutePosition(), null);
        builder.addDoubleProperty("Position", () -> cc.getPosition(), p -> cc.setPosition(p));
        builder.addDoubleProperty("Velocity", () -> cc.getVelocity(), null);
    }

}
