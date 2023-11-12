// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team871;

import com.team871.modules.SwerveDrive;
import com.team871.modules.SwerveModule;
import com.team871.modules.SwerveModule.ModulePosition;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  SwerveDrive drive;
  XboxController joy;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    this.drive = new SwerveDrive(tab,
      new SwerveModule(ModulePosition.RearLeft, 1, 3, 2, 
        tab.getLayout("RearLeft", BuiltInLayouts.kGrid)
         // .withPosition(0 , 4)
          .withSize(5,4)),
      new SwerveModule(ModulePosition.FrontLeft, 4, 6, 5, 
        tab.getLayout("FrontLeft", BuiltInLayouts.kGrid)
          //.withPosition(0 , 0)
          .withSize(5,4)),
      new SwerveModule(ModulePosition.FrontRight, 7, 9, 8, 
        tab.getLayout("FrontRight", BuiltInLayouts.kGrid)
          //.withPosition(5 , 0)
          .withSize(5,4)),
      new SwerveModule(ModulePosition.RearRight, 10, 12, 11, 
        tab.getLayout("RearRight", BuiltInLayouts.kGrid)
         // .withPosition(5 , 4)
          .withSize(5,4)));

    joy = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {
    drive.drive(-joy.getLeftY(), joy.getLeftX(), joy.getRightX());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
