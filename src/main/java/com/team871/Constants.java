package com.team871;

import com.team871.modules.SwerveModule.ModulePosition;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    private Constants() {
    }

    public static final double WHEEL_DIAM_IN = 4;
    public static final double WHEEL_LENGTH_IN = WHEEL_DIAM_IN * Math.PI;
    public static final double WHEEL_LENGTH_METERS = Units.inchesToMeters(WHEEL_LENGTH_IN);
    public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double STEERING_GEAR_RATION = (14.0 / 50.0) * (10.0 / 60.0);

    public static double getMagnetOffset(ModulePosition modPos) {
        switch (modPos) {
            case FrontLeft:
                return 0;
            case FrontRight:
                return 0;
            case RearLeft:
                return 0;
            case RearRight:
                return 0;
            default:
                return 0;
        }
    }

    public static boolean getSensorDirection(ModulePosition modPos) {
        switch (modPos) {
            case FrontLeft:
                return true;
            case FrontRight:
                return true;
            case RearLeft:
                return false;
            case RearRight:
                return false;
            default:
                return false;
        }
    }

    public static Translation2d getLeverArm(ModulePosition modPos) {
        switch (modPos) {
            case FrontLeft:
                return new Translation2d(-.8, .8);
            case FrontRight:
                return new Translation2d(.8, .8);
            case RearLeft:
                return new Translation2d(-.8, -.8);
            case RearRight:
                return new Translation2d(.8, -.8);
            default:
                return null;

        }
    }
}
