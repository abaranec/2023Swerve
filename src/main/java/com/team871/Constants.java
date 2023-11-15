package com.team871;

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

    public static final int FL_DRIVE_CAN_ID = 6;
    public static final int FL_STEER_CAN_ID = 4;
    public static final int FL_SENSOR_CAN_ID = 5;
    public static final double FL_ANGLE_OFFSET = 54.66796875;
    public static final boolean FL_DRIVE_INVERTED = false;
    public static final boolean FL_STEER_INVERTED = true;
    public static final boolean FL_SENSOR_INVERTED = false;
    public static final Translation2d FL_LEVER_ARM = new Translation2d(.3556, .3556);

    public static final int FR_DRIVE_CAN_ID = 9;
    public static final int FR_STEER_CAN_ID = 7;
    public static final int FR_SENSOR_CAN_ID = 8;
    public static final double FR_ANGLE_OFFSET = 262.09;
    public static final boolean FR_DRIVE_INVERTED = false;
    public static final boolean FR_STEER_INVERTED = true;
    public static final boolean FR_SENSOR_INVERTED = false;
    public static final Translation2d FR_LEVER_ARM = new Translation2d(.3556, -.3556);

    public static final int BL_DRIVE_CAN_ID = 3;
    public static final int BL_STEER_CAN_ID = 1;
    public static final int BL_SENSOR_CAN_ID = 2;
    public static final double BL_ANGLE_OFFSET = 23.5546875;
    public static final boolean BL_DRIVE_INVERTED = false;
    public static final boolean BL_STEER_INVERTED = true;
    public static final boolean BL_SENSOR_INVERTED = false;
    public static final Translation2d BL_LEVER_ARM = new Translation2d(-.3556, .3556);

    public static final int RR_DRIVE_CAN_ID = 12;
    public static final int RR_STEER_CAN_ID = 10;
    public static final int RR_SENSOR_CAN_ID = 11;
    public static final double RR_ANGLE_OFFSET = 355.78;
    public static final boolean RR_DRIVE_INVERTED = false;
    public static final boolean RR_STEER_INVERTED = true;
    public static final boolean RR_SENSOR_INVERTED = false;
    public static final Translation2d RR_LEVER_ARM = new Translation2d(-.3556, -.3556);
}
