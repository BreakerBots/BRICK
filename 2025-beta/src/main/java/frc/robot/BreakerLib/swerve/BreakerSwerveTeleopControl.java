// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import java.util.Optional;

import javax.print.DocPrintJob;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;

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
    if (teleopControlConfig.getMaxModuleAzimuthVelocity().isPresent() && drivetrain.constants.pathplannerConfig.robotConfig.isPresent()) {
      setpointGenerator = Optional.of(new SwerveSetpointGenerator(drivetrain.constants.pathplannerConfig.robotConfig.get(), teleopControlConfig.getMaxModuleAzimuthVelocity().get()));
    } else {
      setpointGenerator = Optional.empty();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   headingSetpoint = drivetrain.getPigeon2().getRotation2d();
   lastTimestamp
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

    if (setpointGenerator.isPresent())


    request.withVelocityX(xImpt).withVelocityY(yImpt).withRotationalRate(omegaImpt);
    drivetrain.setControl(request);
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
    private Optional<AngularVelocity> maxModuleAzimuthVelocity;

    public TeleopControlConfig withHeadingCompensation(HeadingCompensationConfig headingCompensationConfig) {
      this.headingCompensationConfig = Optional.of(headingCompensationConfig);
      return this;
    }

    public TeleopControlConfig withSetpointGeneration(AngularVelocity maxModuleAzimuthVelocity) {
      this.maxModuleAzimuthVelocity = Optional.of(maxModuleAzimuthVelocity);
      return this;
    }

    public Optional<HeadingCompensationConfig> getHeadingCompensationConfig() {
        return headingCompensationConfig;
    }

    public Optional<AngularVelocity> getMaxModuleAzimuthVelocity() {
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
