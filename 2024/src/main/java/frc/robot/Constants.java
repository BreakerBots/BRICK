// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.util.loging.BreakerLog.GitInfo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class GeneralConstants {
                public static final String DRIVE_CANIVORE_NAME = "drive_canivore";
                public static final GitInfo GIT_INFO = new GitInfo(BuildConstants.MAVEN_NAME,
                                BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE,
                                BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
        }

        public static class OperatorConstants {
                public static final int CONTROLLER_PORT = 0;
                public static final double TRANSLATIONAL_DEADBAND = 0.075;
                public static final double ROTATIONAL_DEADBAND = 0.075;
        }

        public static class DriveConstants {
                public static final HeadingCompensationConfig HEADING_COMPENSATION_CONFIG = new HeadingCompensationConfig(
                                Units.MetersPerSecond.of(0.05), Units.RadiansPerSecond.of(0.001),
                                new PIDConstants(0, 0, 0));// 2.8
                public static final Measure<Velocity<Distance>> MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond
                                .of(4.5);
                public static final Measure<Velocity<Angle>> MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond
                                .of(9.5);
                 // The steer motor uses any SwerveModule.SteerRequestType control request with the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0).withKV(0.2).withKA(0);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final double kSlipCurrentA = 80.0;

                // Initial configs for the drive and steer motors and the CANcoder; these cannot
                // be null.
                // Some configs will be overwritten; check the `with*InitialConfigs()` API
                // documentation.
                private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
                private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                                .withCurrentLimits(
                                                new CurrentLimitsConfigs()
                                                                // Swerve azimuth does not require much torque output,
                                                                // so we can set a relatively low
                                                                // stator current limit to help avoid brownouts without
                                                                // impacting performance.
                                                                .withStatorCurrentLimit(60)
                                                                .withStatorCurrentLimitEnable(true));
                private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
                // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
                private static final Pigeon2Configuration pigeonConfigs = null;

                // Theoretical free speed (m/s) at 12v applied output;
                // This needs to be tuned to your individual robot
                public static final double kSpeedAt12VoltsMps = 0;

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 3.125;

                private static final double kDriveGearRatio = 5.357142857142857;
                private static final double kSteerGearRatio = 21.428571428571427;
                private static final double kWheelRadiusInches = 2.0;

                private static final String kCANbusName = GeneralConstants.DRIVE_CANIVORE_NAME;
                private static final int kPigeonId = 5;

                // These are only used for simulation
                private static final double kSteerInertia = 0.00001;
                private static final double kDriveInertia = 0.001;
                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.25;
                private static final double kDriveFrictionVoltage = 0.25;

                public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
                                .withCANbusName(kCANbusName)
                                .withPigeon2Id(kPigeonId)
                                .withPigeon2Configs(pigeonConfigs);

                private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withWheelRadius(kWheelRadiusInches)
                                .withSlipCurrent(kSlipCurrentA)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withDriveMotorInitialConfigs(driveInitialConfigs)
                                .withSteerMotorInitialConfigs(steerInitialConfigs)
                                .withCANcoderInitialConfigs(cancoderInitialConfigs);

                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;
                // Front Left
                private static final int kFrontLeftDriveMotorId = 10;
                private static final int kFrontLeftSteerMotorId = 11;
                private static final int kFrontLeftEncoderId = 20;
                private static final double kFrontLeftEncoderOffset = 0.186279296875;
                private static final boolean kFrontLeftSteerInvert = true;
                private static final Translation2d kFrontLeftModulePosition = new Translation2d(0.314325, 0.314325);

                // Front Right
                private static final int kFrontRightDriveMotorId = 12;
                private static final int kFrontRightSteerMotorId = 13;
                private static final int kFrontRightEncoderId = 21;
                private static final double kFrontRightEncoderOffset = -0.09716796875;
                private static final boolean kFrontRightSteerInvert = true;

                private static final Translation2d kFrontRightModulePosition = new Translation2d(0.314325, -0.314325);

                // Back Left
                private static final int kBackLeftDriveMotorId = 14;
                private static final int kBackLeftSteerMotorId = 15;
                private static final int kBackLeftEncoderId = 22;
                private static final double kBackLeftEncoderOffset = 0.3154296875;
                private static final boolean kBackLeftSteerInvert = true;

                private static final Translation2d kBackLeftModulePosition = new Translation2d(-0.314325, 0.314325);

                // Back Right
                private static final int kBackRightDriveMotorId = 16;
                private static final int kBackRightSteerMotorId = 17;
                private static final int kBackRightEncoderId = 23;
                private static final double kBackRightEncoderOffset = -0.302978515625;
                private static final boolean kBackRightSteerInvert = true;
                private static final Translation2d kBackRightModulePosition = new Translation2d(-0.314325, -0.314325);

                public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator
                                .createModuleConstants(
                                                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                                                kFrontLeftEncoderOffset, kFrontLeftModulePosition.getX(),
                                                kFrontLeftModulePosition.getY(), kInvertLeftSide)
                                                .withSteerMotorInverted(kFrontLeftSteerInvert);

                public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator
                                .createModuleConstants(
                                                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                                                kFrontRightEncoderOffset, kFrontRightModulePosition.getX(),
                                                kFrontRightModulePosition.getY(), kInvertRightSide)
                                                .withSteerMotorInverted(kFrontRightSteerInvert);

                public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator
                                .createModuleConstants(
                                                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
                                                kBackLeftEncoderOffset, kBackLeftModulePosition.getX(),
                                                kBackLeftModulePosition.getY(), kInvertLeftSide)
                                                .withSteerMotorInverted(kBackLeftSteerInvert);

                public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator
                                .createModuleConstants(
                                                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                                                kBackRightEncoderOffset, kBackRightModulePosition.getX(),
                                                kBackRightModulePosition.getY(), kInvertRightSide)
                                                .withSteerMotorInverted(kBackRightSteerInvert);

        }
}
