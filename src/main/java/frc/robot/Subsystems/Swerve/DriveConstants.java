// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double maxSpeedMetersPerSec = 0;
        public static final double odometryFrequency =0.0; // Hz
        public static final double trackWidth = 0.0;
        public static final double wheelBase = 0.0;
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        // Zeroed rotation values for each module, see setup instructions
        public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0);
        public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
        public static final Rotation2d backLeftZeroRotation = new Rotation2d(0);
        public static final Rotation2d backRightZeroRotation = new Rotation2d(0);

        // Device CAN IDs
        public static final int pigeonCanId = 2;

        public static final int swerveBaseID = 3;
        public static final int swerveModuleIDsCount = 3;

        // public static final int frontLeftDriveCanId = 3;
        // public static final int backLeftDriveCanId = 5;
        // public static final int frontRightDriveCanId = 7;
        // public static final int backRightDriveCanId = 9;

        // public static final int frontLeftTurnCanId = 4;
        // public static final int backLeftTurnCanId = 6;
        // public static final int frontRightTurnCanId = 8;
        // public static final int backRightTurnCanId = 10;

        public static final int frontLeftDefaultCanId = 3;
        public static final int backLeftDefaultCanId = 9;
        public static final int frontRightDefaultCanId = 6;
        public static final int backRightDefaultCanId = 13;

        // Drive motor configuration
        public static final int driveMotorCurrentLimit = 0;
        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double driveMotorReduction = 0; // Change to 5.96 (or 5.46) if using faster ration
        public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

        // Drive encoder configuration
        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
                                                                                                   // Wheel Radians
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
                                                                                                            // ->
                                                                                                            // Wheel
                                                                                                            // Rad/Sec

        // Drive PID configuration
        public static final double driveKp = 0.0;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.0;
        public static final double driveKv = 0.0;
        public static final double driveSimP = 0.0;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0;

        public static final double driveSlipCurrent = 0.0;
        public static final double driveRampRate = 0.0;

        // Turn motor configuration
        public static final boolean turnInverted = true;
        public static final int turnMotorCurrentLimit = 0;
        public static final double turnMotorRampRate = 0.0;
        public static final double turnMotorReduction = 396.0 / 35.0;
        public static final DCMotor turnGearbox = DCMotor.getNEO(1);

        // Turn encoder configuration
        public static final boolean turnEncoderInverted = true;
        public static final double turnEncoderPositionFactor = (2 * Math.PI) / turnMotorReduction; // Rotations ->
                                                                                                   // Radians
        public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60.0; // RPM -> Rad/Sec

        // Turn PID configuration
        public static final double turnKp = 0.0;
        public static final double turnKd = 0.0;
        public static final double turnKs = 0.0;
        // public static final double turnKp = 0.275;
        // public static final double turnKd = 0.0744;
        public static final double turnSimP = 0.0;
        public static final double turnSimD = 0.0;
        public static final double turnPIDMinInput = 0; // Radians
        public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

        // PathPlanner configuration
        public static final double robotMassKg = 0;
        public static final double robotMOI = 1 / 12.0 * robotMassKg * (2 * trackWidth * trackWidth);
        public static final double wheelCOF = 1.2;
        // public static final RobotConfig ppConfig = new RobotConfig( //FIXME dont working 
        //                 robotMassKg,
        //                 robotMOI,
        //                 new ModuleConfig(
        //                                 wheelRadiusMeters,
        //                                 maxSpeedMetersPerSec,
        //                                 wheelCOF,
        //                                 driveGearbox.withReduction(driveMotorReduction),
        //                                 driveMotorCurrentLimit,
        //                                 1),
        //                 moduleTranslations);
}
