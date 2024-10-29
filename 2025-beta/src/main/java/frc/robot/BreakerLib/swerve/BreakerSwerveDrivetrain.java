// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import static java.lang.Math.abs;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.proto.Twist2dProto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class BreakerSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  /** Creates a new BreakerSwerveDrivetrain. */
  protected Notifier simNotifier = null;
  protected double lastSimTime;
  protected final double commonMaxModuleSpeed;
  protected final BreakerSwerveDrivetrainConstants constants;
  protected Consumer<SwerveDriveState> userTelemetryCallback = null;
  /* Keep track if we've ever applied the operator perspective before or not */
  protected boolean hasAppliedOperatorPerspective = false;
  protected ChassisSpeeds prevChassisSpeeds = new ChassisSpeeds();
  protected ChassisAccels chassisAccels = new ChassisAccels();

  protected AutoFactory autoFactory;

  public BreakerSwerveDrivetrain(
    BreakerSwerveDrivetrainConstants driveTrainConstants, 
    SwerveModuleConstants... modules
  ) {
    this(driveTrainConstants,
    VecBuilder.fill(0.1, 0.1, 0.1),
    VecBuilder.fill(0.9, 0.9, 0.9), 
    modules);
  }

  public BreakerSwerveDrivetrain(
      BreakerSwerveDrivetrainConstants driveTrainConstants, 
      Matrix<N3, N1> odometryStandardDeviation, 
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules
    ) {
    super(driveTrainConstants, driveTrainConstants.odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    this.constants = driveTrainConstants;
    m_telemetryFunction = this::telemetryCallbackWrapperFunction;
    double tempCommonMaxModuleSpeed = Double.MAX_VALUE;
    for (SwerveModuleConstants modConst : modules) {
      tempCommonMaxModuleSpeed = Math.min(tempCommonMaxModuleSpeed, modConst.SpeedAt12Volts); //@TODO this uses the 12v nominal speed because a direct max speed is not yet exposed
    }
    commonMaxModuleSpeed = tempCommonMaxModuleSpeed;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configPathPlanner();
    configChoreo();
  }
  
  private void telemetryCallbackWrapperFunction(SwerveDriveState state) {
    chassisAccels = ChassisAccels.fromDeltaSpeeds(prevChassisSpeeds, state.Speeds, state.OdometryPeriod);
    prevChassisSpeeds = state.Speeds;
    
    BreakerLog.log("SwerveDrivetrain/State/Pose", state.Pose);
    BreakerLog.log("SwerveDrivetrain/State/Speeds", state.Speeds);
    BreakerLog.log("SwerveDrivetrain/State/Accels", chassisAccels);
    BreakerLog.log("SwerveDrivetrain/State/ModuleStates", state.ModuleStates);
    BreakerLog.log("SwerveDrivetrain/State/TargetModuleStates", state.ModuleTargets);
    BreakerLog.log("SwerveDrivetrain/State/SuccessfulDAQs", state.SuccessfulDaqs);
    BreakerLog.log("SwerveDrivetrain/State/FailedDAQs", state.FailedDaqs);
    BreakerLog.log("SwerveDrivetrain/State/OdometryPeriod", state.OdometryPeriod);
    
    if (userTelemetryCallback != null) {
      userTelemetryCallback.accept(state);
    }
  }

  private void lowFrequencyTelemetry() {
    BreakerLog.log("SwerveDrivetrain/Modules", getModules());
    BreakerLog.log("SwerveDrivetrain/Pigeon2", getPigeon2());
  }
  
    /**
   * Register the specified lambda to be executed whenever our SwerveDriveState function
   * is updated in our odometry thread.
   * <p>
   * It is imperative that this function is cheap, as it will be executed along with
   * the odometry call, and if this takes a long time, it may negatively impact
   * the odometry of this stack.
   * <p>
   * This can also be used for logging data if the function performs logging instead of telemetry
   *
   * @param telemetryFunction Function to call for telemetry or logging
   */
  public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
    try {
        m_stateLock.lock();
        userTelemetryCallback = telemetryFunction;
    } finally {
        m_stateLock.unlock();
    }
  }

  public ChassisAccels getChassisAccels() {
    try {
      m_stateLock.lock();
      return chassisAccels;
    } finally {
      m_stateLock.unlock();
    }
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return  m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getCurrentFieldRelitiveChassisSpeeds() {
    return  BreakerMath.fromRobotRelativeSpeeds(getState().Speeds, getState().Pose.getRotation());
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void configPathPlanner() {
    if (constants.pathplannerConfig.robotConfig.isPresent()) {
      double driveBaseRadius = 0;
      for (var moduleLocation : m_moduleLocations) {
          driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
      }

      SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
      request.DriveRequestType = DriveRequestType.Velocity;
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output = (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
        request.Speeds = speeds;
        request.WheelForceFeedforwardsX = feedforwards.robotRelativeForcesXNewtons();
        request.WheelForceFeedforwardsY = feedforwards.robotRelativeForcesYNewtons();
        setControl(request);
      };

      AutoBuilder.configure(
        ()->this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative,  // Consumer for seeding pose against auto
        this::getCurrentChassisSpeeds,
        output, // Consumer of ChassisSpeeds to drive the robot
        new PPHolonomicDriveController(constants.pathplannerConfig.translationPID, constants.pathplannerConfig.rotationPID),
        constants.pathplannerConfig.robotConfig.get(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        this); // Subsystem for requirements
    }
  }

  private void configChoreo() {
    PIDController x = new PIDController(constants.choreoConfig.translationPID.kP, constants.choreoConfig.translationPID.kI, constants.choreoConfig.translationPID.kD);
    PIDController y = new PIDController(constants.choreoConfig.translationPID.kP, constants.choreoConfig.translationPID.kI, constants.choreoConfig.translationPID.kD);
    PIDController r = new PIDController(constants.choreoConfig.rotationPID.kP, constants.choreoConfig.rotationPID.kI, constants.choreoConfig.rotationPID.kD);
    autoFactory = Choreo.createAutoFactory(
      this, 
      () -> this.getState().Pose, 
      new BreakerSwerveChoreoController(this, x, y, r),
      () -> {return DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red;}, 
      constants.choreoConfig.autoBindings, 
      this::logChoreoPath);
  }

  protected void logChoreoPath(Trajectory<SwerveSample> trajectory, boolean isStarting) {
    BreakerLog.log("Choreo/LastTrajectory", trajectory);
    BreakerLog.log("Choreo/LastTrajectory/State", isStarting ? "Starting" : "Finishing");
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(1.0/constants.simUpdateFrequency);
  }

  public Rotation2d getOperatorForwardDirection() {
    return m_controlParams.operatorForwardDirection;
  }


  public BreakerSwerveTeleopControl getTeleopControlCommand(BreakerInputStream x, BreakerInputStream y, BreakerInputStream omega, TeleopControlConfig teleopControlConfig) {
    return new BreakerSwerveTeleopControl(this, x, y, omega, teleopControlConfig);
  }

  @Override
  public void periodic() {
    lowFrequencyTelemetry();
   /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
          DriverStation.getAlliance().ifPresent((allianceColor) -> {
              this.setOperatorPerspectiveForward(
                      allianceColor == Alliance.Red ? constants.redAlliancePerspectiveRotation
                              : constants.redAlliancePerspectiveRotation);
              hasAppliedOperatorPerspective = true;
        });
      }
      
  }

  public static class BreakerSwerveDrivetrainConstants extends SwerveDrivetrainConstants {
    public double odometryUpdateFrequency = 250;
    public double simUpdateFrequency = 200;
    public Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    public Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    public ChoreoConfig choreoConfig = new ChoreoConfig();
    public PathplannerConfig pathplannerConfig = new PathplannerConfig();

    public BreakerSwerveDrivetrainConstants() {
      super();
    }

    @Override
    public BreakerSwerveDrivetrainConstants withCANBusName(String name) {
      this.CANBusName = name;
      return this;
    }

    @Override
    public BreakerSwerveDrivetrainConstants withPigeon2Configs(Pigeon2Configuration configs) {
      this.Pigeon2Configs = configs;
      return this;
    }

    @Override
    public BreakerSwerveDrivetrainConstants withPigeon2Id(int id) {
      this.Pigeon2Id = id;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withOdometryUpdateFrequency(double frequencyHz) {
      odometryUpdateFrequency = frequencyHz;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withSimUpdateFrequency(double frequencyHz) {
      simUpdateFrequency = frequencyHz;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withBlueAlliancePerspectiveRotation(Rotation2d perspectiveRotation) {
      blueAlliancePerspectiveRotation = perspectiveRotation;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withRedAlliancePerspectiveRotation(Rotation2d perspectiveRotation) {
      redAlliancePerspectiveRotation = perspectiveRotation;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withChoreoConfig(ChoreoConfig choreoConfig) {
      this.choreoConfig = choreoConfig;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withChoreoConfig(PathplannerConfig pathplannerConfig) {
      this.pathplannerConfig = pathplannerConfig;
      return this;
    }

    public static class ChoreoConfig {
      public PIDConstants translationPID = new PIDConstants(10, 0, 0);
      public PIDConstants rotationPID = new PIDConstants(10, 0, 0);
      public AutoBindings autoBindings = new AutoBindings();
      public ChoreoConfig() {}

      public ChoreoConfig withTranslationPID(PIDConstants translationPID) {
        this.translationPID = translationPID;
        return this;
      }

      public ChoreoConfig withRotationPID(PIDConstants rotationPID) {
        this.rotationPID = rotationPID;
        return this;
      }

      public ChoreoConfig withAutoBindings(AutoBindings autoBindings) {
        this.autoBindings = autoBindings;
        return this;
      }
    }

    public static class PathplannerConfig {
      public PIDConstants translationPID = new PIDConstants(10, 0, 0);
      public PIDConstants rotationPID = new PIDConstants(10, 0, 0);
      public Optional<RobotConfig> robotConfig = getRobotConfigFromGUI();
      public PathplannerConfig() {
      }

      private Optional<RobotConfig> getRobotConfigFromGUI() {
        try {
          return Optional.of(RobotConfig.fromGUISettings());
        } catch(Exception e) {
          DriverStation.reportError("Failed to load RobotConfig from PathPlanner GUI", true);
          return Optional.empty();
        }
      }


      public PathplannerConfig withTranslationPID(PIDConstants translationPID) {
        this.translationPID = translationPID;
        return this;
      }

      public PathplannerConfig withRotationPID(PIDConstants rotationPID) {
        this.rotationPID = rotationPID;
        return this;
      }

      public PathplannerConfig withRobotConfig(RobotConfig robotConfig) {
        this.robotConfig = Optional.of(robotConfig);
        return this;
      }
    }
  } 
}
