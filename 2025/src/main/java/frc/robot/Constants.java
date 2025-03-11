// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
// import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.ChoreoConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.logging.BreakerLog.GitInfo;
import frc.robot.commands.AutoPilot.NavToPoseConfig;
import frc.robot.commands.AutoPilot.ProfiledPIDControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class ApriltagVisionConstants {
      public static final String kTopLeftCameraName = "top_left";
      public static final String kTopRightCameraName = "top_right";
      public static final String kBottomLeftCameraName = "bottom_left";
      public static final String kBottomRightCameraName = "bottom_right";

      public static final Transform3d kTopLeftCameraTransform = new Transform3d(new Translation3d(Inches.of(-9.48),Inches.of(10.54),Inches.of(37.486).plus(Inches.of(1.544))), new Rotation3d(Degrees.of(0), Degrees.of(-15),Degrees.of(145)));
      public static final Transform3d kTopRightCameraTransform = new Transform3d(new Translation3d(Inches.of(-9.48),Inches.of(-10.54),Inches.of(37.486).plus(Inches.of(1.544))), new Rotation3d(Degrees.of(0), Degrees.of(-15),Degrees.of(-145)));
      public static final Transform3d kBottomLeftCameraTransform = new Transform3d(new Translation3d(Inches.of(-11.642),Inches.of(10.425),Inches.of(6.761).plus(Inches.of(1.544))), new Rotation3d(Degrees.of(0), Degrees.of(-20),Degrees.of(-170).plus(Degrees.of(5))));//10.425 //Degrees.of(-165).minus(Degrees.of(2.5))//25
      public static final Transform3d kBottomRightCameraTransform = new Transform3d(new Translation3d(Inches.of(-11.642),Inches.of(-10.425),Inches.of(6.761).plus(Inches.of(1.544))), new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(170).minus(Degrees.of(5))));//-10.425//Degrees.of(165).plus(Degrees.of(2.5))//25

      public static final SimCameraProperties kBottomLeftCameraSimProperties = getThriftyCam()
        .setCalibError(0.9, 0.001)
        .setFPS(40)
        .setAvgLatencyMs(35)
        .setLatencyStdDevMs(3);

      public static final SimCameraProperties kBottomRightCameraSimProperties =
        getThriftyCam()
        .setCalibError(0.9, 0.001)
        .setFPS(40)
        .setAvgLatencyMs(35)
        .setLatencyStdDevMs(3);

      
      public static final Distance kMaxTrigSolveTagDist = Meters.of(2.5);
      public static final Matrix<N3, N1> kTrigBaseStdDevs = VecBuilder.fill(0.5, 0.5, 15);
      public static final double kTrigDevScaleFactor = 5;

      public static SimCameraProperties getThriftyCam() {
        return new SimCameraProperties()
        .setCalibration(
          1600, 
          1304,
          MatBuilder.fill(
            Nat.N3(), 
            Nat.N3(), 
            1380.278298417611,
            0.0,
            812.9866295000404,
            0.0,
            1379.4771633563626,
            713.7349392103608,
            0.0,
            0.0,
            1.0
            ),
          VecBuilder.fill(
            -0.023583816443651925,
            -0.013927876662786186,
            -5.265726756324146E-4,
            -1.1610575615885912E-4,
            0.03363075302770153,
            4.900879121679141E-4,
            2.539986658798725E-4,
            -0.0012091457686458247
          )
        );
      }
      
    }

    public static class FieldConstants {
      public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Distance kReefBranchOffsetFromFaceApriltagStrafe = Inches.of(6.47);
        public static final Distance kReefFaceLength = Inches.of(36.792600);
    }

    public static class AutoPilotConstants {
      public static final Distance kReefAutoAllignOffsetFromReefFace = Inches.of(24);

      public static final  ProfiledPIDControllerConfig kDefaultTranslationConfig = new ProfiledPIDControllerConfig(4.5, 0.001, 0, new Constraints(1.0, 1.5));
      public static final  ProfiledPIDControllerConfig kDefaultRotationConfig = new ProfiledPIDControllerConfig(3.5, 0, 0, new Constraints(2.0, 5.0));

      public static final NavToPoseConfig kDefaultNavToPoseConfig = new NavToPoseConfig(
        true,
        new Pose2d(0.025, 0.025, Rotation2d.fromDegrees(2)),
        new ChassisSpeeds(0.15, 0.15, 0.015), 
        kDefaultTranslationConfig, 
        kDefaultTranslationConfig, 
        kDefaultRotationConfig);

      public static final  ProfiledPIDControllerConfig kAutoTranslationConfig = new ProfiledPIDControllerConfig(4.4, 0.001, 0, new Constraints(1.5, 3));
      public static final  ProfiledPIDControllerConfig kAutoRotationConfig = new ProfiledPIDControllerConfig(3.5, 0, 0, new Constraints(2.0, 5.0));

      public static final NavToPoseConfig kAutoNavToPoseConfig = new NavToPoseConfig(
        true,
        new Pose2d(0.025, 0.025, Rotation2d.fromDegrees(2)),
        new ChassisSpeeds(0.15, 0.15, 0.015), 
        kAutoTranslationConfig, 
        kAutoTranslationConfig, 
        kAutoRotationConfig);
    }

  public static class GeneralConstants {
    public static final CANBus DRIVE_CANIVORE_BUS = new CANBus("drive_canivore");
    public static final GitInfo GIT_INFO = new GitInfo(BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
  }
  
  public static class OperatorConstants {
    public static final int CONTROLLER_PORT= 0;
    public static final double TRANSLATIONAL_DEADBAND = 0.1;
     public static final double ROTATIONAL_DEADBAND = 0.1;
  }

  public static class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.5, 0, 0.8);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.5, 0,1);
        public static final ChoreoConfig CHOREO_CONFIG = new ChoreoConfig().withTranslationPID(TRANSLATION_PID).withRotationPID(ROTATION_PID);
  }

    public static class DriveConstants {
                public static final AngularVelocity MAXIMUM_MODULE_AZIMUTH_SPEED = Units.DegreesPerSecond.of(720);
                public static final HeadingCompensationConfig HEADING_COMPENSATION_CONFIG = new HeadingCompensationConfig(
                                Units.MetersPerSecond.of(0.05), 
                                Units.RadiansPerSecond.of(0.001), 
                                Units.Seconds.of(0.2),
                                new PIDConstants(1.5, 0, 0));// 1.5
                //public static final SetpointGenerationConfig SETPOINT_GENERATION_CONFIG = new SetpointGenerationConfig(MAXIMUM_MODULE_AZIMUTH_SPEED);
                public static final TeleopControlConfig TELEOP_CONTROL_CONFIG = new TeleopControlConfig()
                        .withHeadingCompensation(HEADING_COMPENSATION_CONFIG);
                        //.withSetpointGeneration(SETPOINT_GENERATION_CONFIG);
                public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(4.5);
                public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(9.5);
                 // The steer motor uses any SwerveModule.SteerRequestType control request with the
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.01).withKI(0).withKD(0)
                .withKS(0.005).withKV(0.15).withKA(0.01);

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final Current kSlipCurrent = Amps.of(80.0);

                private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
                private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
                private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

                // Initial configs for the drive and steer motors and the CANcoder; these cannot
                // be null.
                // Some configs will be overwritten; check the `with*InitialConfigs()` API
                // documentation.
                private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
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
                public static final LinearVelocity kSpeedAt12Volts = Units.MetersPerSecond.of(0.0);

                // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
                // This may need to be tuned to your individual robot
                private static final double kCoupleRatio = 3.125;

                private static final double kDriveGearRatio = 5.357142857142857;
                private static final double kSteerGearRatio = 21.428571428571427;
                private static final Distance kWheelRadius = Units.Inches.of(2.0);

                private static final String kCANbusName = GeneralConstants.DRIVE_CANIVORE_BUS.getName();
                private static final int kPigeonId = 5;

                // These are only used for simulation
                private static final double kSteerInertia = 0.00001;
                private static final double kDriveInertia = 0.001;
                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.25;
                private static final double kDriveFrictionVoltage = 0.25;

                public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
                                .withCANBusName(kCANbusName)
                                .withPigeon2Id(kPigeonId)
                                .withPigeon2Configs(pigeonConfigs)
                                .withChoreoConfig(AutoConstants.CHOREO_CONFIG);

                                private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
                                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                                    .withDriveMotorGearRatio(kDriveGearRatio)
                                    .withSteerMotorGearRatio(kSteerGearRatio)
                                    .withCouplingGearRatio(kCoupleRatio)
                                    .withWheelRadius(kWheelRadius)
                                    .withSteerMotorGains(steerGains)
                                    .withDriveMotorGains(driveGains)
                                    .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                    .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                    .withSlipCurrent(kSlipCurrent)
                                    .withSpeedAt12Volts(kSpeedAt12Volts)
                                    .withDriveMotorType(kDriveMotorType)
                                    .withSteerMotorType(kSteerMotorType)
                                    .withFeedbackSource(kSteerFeedbackType)
                                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                                    .withEncoderInitialConfigs(cancoderInitialConfigs)
                                    .withSteerInertia(kSteerInertia)
                                    .withDriveInertia(kDriveInertia)
                                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

                private static final boolean kInvertLeftSide = false;
                private static final boolean kInvertRightSide = true;
                // Front Left
                private static final int kFrontLeftDriveMotorId = 10;
                private static final int kFrontLeftSteerMotorId = 11;
                private static final int kFrontLeftEncoderId = 20;
                private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.186279296875)        ;
                private static final boolean kFrontLeftSteerInvert = true;
                private static final boolean kFrontLeftEncoderInvert = false;
                private static final Translation2d kFrontLeftModulePosition = new Translation2d(
                        Units.Inches.of(9.84),
                        Units.Inches.of(9.84)
                );

                // Front Right
                private static final int kFrontRightDriveMotorId = 12;
                private static final int kFrontRightSteerMotorId = 13;
                private static final int kFrontRightEncoderId = 21;
                private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.09716796875);
                private static final boolean kFrontRightSteerInvert = true;
                private static final boolean kFrontRightEncoderInvert = false;

                private static final Translation2d kFrontRightModulePosition = new Translation2d(
                        Units.Inches.of(9.84),
                        Units.Inches.of(-9.84)
                );

                // Back Left
                private static final int kBackLeftDriveMotorId = 14;
                private static final int kBackLeftSteerMotorId = 15;
                private static final int kBackLeftEncoderId = 22;
                private static final Angle kBackLeftEncoderOffset = Rotation.of(0.3154296875);
                private static final boolean kBackLeftSteerInvert = true;
                private static final boolean kBackLeftEncoderInvert = false;

                private static final Translation2d kBackLeftModulePosition = new Translation2d(
                        Units.Inches.of(-9.84),
                        Units.Inches.of(9.84)
                );

                // Back Right
                private static final int kBackRightDriveMotorId = 16;
                private static final int kBackRightSteerMotorId = 17;
                private static final int kBackRightEncoderId = 23;
                private static final Angle kBackRightEncoderOffset = Rotations.of(-0.302978515625);
                private static final boolean kBackRightSteerInvert = true;
                private static final boolean kBackRightEncoderInvert = false;
                private static final Translation2d kBackRightModulePosition = new Translation2d(
                        Units.Inches.of(-9.84),
                        Units.Inches.of(-9.84)
                );

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
                ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                    kFrontLeftModulePosition.getMeasureX(), kFrontLeftModulePosition.getMeasureY(), kInvertLeftSide, kFrontLeftSteerInvert, kFrontLeftEncoderInvert
                );
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
                ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                    kFrontRightModulePosition.getMeasureX(), kFrontRightModulePosition.getMeasureY(), kInvertRightSide, kFrontRightSteerInvert, kFrontRightEncoderInvert
                );
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
                ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                    kBackLeftModulePosition.getMeasureX(), kBackLeftModulePosition.getMeasureY(), kInvertLeftSide, kBackLeftSteerInvert, kBackLeftEncoderInvert
                );
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
                ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                    kBackRightModulePosition.getMeasureX(), kBackRightModulePosition.getMeasureY(), kInvertRightSide, kBackRightSteerInvert, kBackRightEncoderInvert
                );
        }


       
}
