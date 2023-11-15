// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.modules.SwerveDrive;
import com.team871.modules.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  SwerveDrive drive;
  SwerveModule[] mods;
  XboxController joy;

  GenericEntry ge;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    ge = tab.add("Desired", 0.0).getEntry();
    mods = new SwerveModule[] {
        new SwerveModule(
            Constants.FL_DRIVE_CAN_ID,
            Constants.FL_DRIVE_INVERTED,
            Constants.FL_STEER_CAN_ID,
            Constants.FL_STEER_INVERTED,
            Constants.FL_SENSOR_CAN_ID,
            Constants.FL_ANGLE_OFFSET,
            Constants.FL_SENSOR_INVERTED,
            Constants.FL_LEVER_ARM,
            tab.getLayout("FrontLeft", BuiltInLayouts.kGrid)
                .withSize(5, 4)),
        new SwerveModule(
            Constants.FR_DRIVE_CAN_ID,
            Constants.FR_DRIVE_INVERTED,
            Constants.FR_STEER_CAN_ID,
            Constants.FR_STEER_INVERTED,
            Constants.FR_SENSOR_CAN_ID,
            Constants.FR_ANGLE_OFFSET,
            Constants.FR_SENSOR_INVERTED,
            Constants.FR_LEVER_ARM,
            tab.getLayout("FrontRight", BuiltInLayouts.kGrid)
                .withSize(5, 4)),
        new SwerveModule(
            Constants.BL_DRIVE_CAN_ID,
            Constants.BL_DRIVE_INVERTED,
            Constants.BL_STEER_CAN_ID,
            Constants.BL_STEER_INVERTED,
            Constants.BL_SENSOR_CAN_ID,
            Constants.BL_ANGLE_OFFSET,
            Constants.BL_SENSOR_INVERTED,
            Constants.BL_LEVER_ARM,
            tab.getLayout("RearLeft", BuiltInLayouts.kGrid)
                .withSize(5, 4)),
        new SwerveModule(
            Constants.RR_DRIVE_CAN_ID,
            Constants.RR_DRIVE_INVERTED,
            Constants.RR_STEER_CAN_ID,
            Constants.RR_STEER_INVERTED,
            Constants.RR_SENSOR_CAN_ID,
            Constants.RR_ANGLE_OFFSET,
            Constants.RR_SENSOR_INVERTED,
            Constants.RR_LEVER_ARM,
            tab.getLayout("RearRight", BuiltInLayouts.kGrid)
                .withSize(5, 4))
    };

    this.drive = new SwerveDrive(tab, mods);

    joy = new XboxController(0);
  }

  private double applyDeadband(double val) {
    return Math.abs(val) < .05 ? 0 : val;
  }

  @Override
  public void teleopPeriodic() {
    drive.drive(
      applyDeadband(-joy.getLeftY()), 
      applyDeadband(-joy.getLeftX()),
      applyDeadband(joy.getRawAxis(2)));
  }

  @Override
  public void testPeriodic() {
    double target = Math.atan2(
        applyDeadband(-joy.getLeftX()),
        applyDeadband(joy.getLeftY()));
    SwerveModuleState newState = new SwerveModuleState(
        new Translation2d(
            applyDeadband(joy.getLeftX()),
            applyDeadband(joy.getLeftY())).getNorm(),
        Rotation2d.fromRadians(target));
    
    mods[0].setState(newState);
    mods[1].setState(newState);
    mods[2].setState(newState);
    mods[3].setState(newState);

    ge.setDouble(Math.toDegrees(target));
  }
}
