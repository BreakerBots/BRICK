// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import static java.lang.Math.random;

import java.io.ObjectInputStream.GetField;
import java.util.Optional;

import javax.print.DocPrintJob;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.physics.BreakerVector2;

public class BreakerSwerveTeleopControl extends Command {
  /** Creates a new BreakerSwerveTeleopControl. */
  private BreakerSwerveDrivetrain drivetrain;
  private BreakerInputStream x, y, omega;
  private SwerveRequest.FieldCentric request;
  private TeleopControlConfig teleopControlConfig;
  private Rotation2d headingSetpoint;
  private Optional<SwerveSetpointGenerator> setpointGenerator;

  private double lastTimestamp;
  private SwerveSetpoint prevSetpoint;
  /**
    Creates a BreakerSwerveTeleopControl command
    @param drivetrain
    @param x 
    @param y
    @param omega
  
  */

  public BreakerSwerveTeleopControl(BreakerSwerveDrivetrain drivetrain, BreakerInputStream x, BreakerInputStream y, BreakerInputStream omega, TeleopControlConfig headingCompensationConfig) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.omega = omega;
    request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    if (teleopControlConfig.getSetpointGenerationConfig().isPresent()) {
      SetpointGenerationConfig generationConfig = teleopControlConfig.getSetpointGenerationConfig().get();
      if (generationConfig.getRobotConfig().isPresent()) {
        setpointGenerator = Optional.of(new SwerveSetpointGenerator(generationConfig.getRobotConfig().get(), generationConfig.getMaxModuleAzimuthVelocity()));
      } else if (drivetrain.constants.pathplannerConfig.robotConfig.isPresent()) {
        setpointGenerator = Optional.of(new SwerveSetpointGenerator(drivetrain.constants.pathplannerConfig.robotConfig.get(), generationConfig.getMaxModuleAzimuthVelocity()));
      } else {
        setpointGenerator = Optional.empty();
      }
    } else {
      setpointGenerator = Optional.empty();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingSetpoint = drivetrain.getPigeon2().getRotation2d();
    lastTimestamp = Timer.getFPGATimestamp();
    prevSetpoint = new SwerveSetpoint(drivetrain.getState().Speeds, drivetrain.getState().ModuleStates, DriveFeedforwards.zeros(drivetrain.getState().ModuleStates.length));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xImpt = x.get();
    double yImpt = y.get();
    double omegaImpt = omega.get();
    if (teleopControlConfig.headingCompensationConfig.isPresent()) {
      HeadingCompensationConfig headingCompensationConfig = teleopControlConfig.headingCompensationConfig.get();
      if (Math.hypot(xImpt, yImpt) >= headingCompensationConfig.getMinActiveLinearVelocity().in(Units.MetersPerSecond) && Math.abs(omegaImpt) > headingCompensationConfig.getAngularVelocityDeadband().in(Units.RadiansPerSecond)) {
        omegaImpt = headingCompensationConfig.getPID().calculate(drivetrain.getPigeon2().getRotation2d().getRadians(), headingSetpoint.getRotations());
      } else {
        headingSetpoint = drivetrain.getPigeon2().getRotation2d();
      }
    }

    if (setpointGenerator.isPresent()) {
      double curTimestamp = Timer.getFPGATimestamp();
      BreakerVector2 linVec = new BreakerVector2(xImpt, yImpt);
      linVec.rotateBy(drivetrain.getOperatorForwardDirection());
      xImpt = linVec.getX();
      yImpt = linVec.getY();
      ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xImpt, yImpt, omegaImpt, drivetrain.getState().Pose.getRotation());
      SwerveSetpoint curSetpoint = setpointGenerator.get().generateSetpoint(prevSetpoint, desiredSpeeds, curTimestamp - lastTimestamp);
      SwerveModuleState[] moduleStates = curSetpoint.moduleStates();
      double[] robotRelativeForcesXNewtons = curSetpoint.feedforwards().robotRelativeForcesXNewtons();
      double[] robotRelativeForcesYNewtons = curSetpoint.feedforwards().robotRelativeForcesYNewtons();
      for (int i = 0; i < curSetpoint.moduleStates().length; i++) {
        SwerveModule module = drivetrain.getModule(i);
        module.apply(new ModuleRequest()
        .withDriveRequest(DriveRequestType.Velocity)
        .withState(moduleStates[i])
        .withWheelForceFeedforwardX(robotRelativeForcesXNewtons[i])
        .withWheelForceFeedforwardY(robotRelativeForcesYNewtons[i])
        );
      }
      lastTimestamp = curTimestamp;
      prevSetpoint = curSetpoint;
    } else {
      request.withVelocityX(xImpt).withVelocityY(yImpt).withRotationalRate(omegaImpt);
      drivetrain.setControl(request);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class TeleopControlConfig {
    private Optional<HeadingCompensationConfig> headingCompensationConfig;
    private Optional<SetpointGenerationConfig> setpointGenerationConfig;

    public TeleopControlConfig withHeadingCompensation(HeadingCompensationConfig headingCompensationConfig) {
      this.headingCompensationConfig = Optional.of(headingCompensationConfig);
      return this;
    }

    public TeleopControlConfig withSetpointGeneration(SetpointGenerationConfig setpointGenerationConfig) {
      this.setpointGenerationConfig = Optional.of(setpointGenerationConfig);
      return this;
    }

    public Optional<HeadingCompensationConfig> getHeadingCompensationConfig() {
        return headingCompensationConfig;
    }

    public Optional<SetpointGenerationConfig> getSetpointGenerationConfig() {
        return setpointGenerationConfig;
    }
  }

  public static class SetpointGenerationConfig {
    private Optional<RobotConfig> robotConfig;
    private AngularVelocity maxModuleAzimuthVelocity;

    public SetpointGenerationConfig(AngularVelocity maxModuleAzimuthVelocity) {
      this.maxModuleAzimuthVelocity = maxModuleAzimuthVelocity;
      robotConfig = Optional.empty();
    }

    public SetpointGenerationConfig(RobotConfig robotConfig, AngularVelocity maxModuleAzimuthVelocity) {
      this(maxModuleAzimuthVelocity);
      this.robotConfig = Optional.of(robotConfig);
    }

    public SetpointGenerationConfig withRobotConfig(RobotConfig robotConfig) {
      this.robotConfig = Optional.of(robotConfig);
      return this;
    }

    public Optional<RobotConfig> getRobotConfig(){
      return robotConfig;
    }

    public AngularVelocity getMaxModuleAzimuthVelocity() {
        return maxModuleAzimuthVelocity;
    }
  }

  public static class HeadingCompensationConfig {
    private LinearVelocity minActiveLinearVelocity;
    private AngularVelocity angularVelocityDeadband;
    private PIDController pid;
    public HeadingCompensationConfig(LinearVelocity minActiveLinearVelocity, AngularVelocity angularVelocityDeadband, PIDConstants pidConstants) {
      this.angularVelocityDeadband = angularVelocityDeadband;
      this.minActiveLinearVelocity = minActiveLinearVelocity;
      pid = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
      pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public AngularVelocity getAngularVelocityDeadband() {
        return angularVelocityDeadband;
    }

    public LinearVelocity getMinActiveLinearVelocity() {
        return minActiveLinearVelocity;
    } 

    public PIDController getPID() {
      return pid;
    }
    
  }
}
