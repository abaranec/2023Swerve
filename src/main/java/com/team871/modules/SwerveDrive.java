package com.team871.modules;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveDrive {
    final SwerveModule[] modules;
    final SwerveDriveKinematics kinematics;
    final GenericEntry tProcEntry;

    public SwerveDrive(final ShuffleboardTab telemetryTab, final SwerveModule... modules) {
        this.modules = modules;
        this.kinematics = new SwerveDriveKinematics(
                Arrays.stream(modules)
                        .map(SwerveModule::getLeverArm)
                        .toArray(Translation2d[]::new));

        this.tProcEntry = telemetryTab.add("tOverall", 0).getEntry();
    }

    public void drive(final double vxMps, final double vyMps, final double rotOmegaPs) {
        final long tStart = RobotController.getFPGATime();
        final SwerveModuleState[] newStates = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(vxMps, vyMps, rotOmegaPs));
        for (int moduleNo = 0; moduleNo < modules.length; moduleNo++) {
            modules[moduleNo].setState(newStates[moduleNo]);
        }
        tProcEntry.setDouble((RobotController.getFPGATime() - tStart) / 1000.0d);
    }
}
