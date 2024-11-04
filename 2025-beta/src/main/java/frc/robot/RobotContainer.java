// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.DRIVETRAIN_CONSTANTS;
import static frc.robot.Constants.DriveConstants.HEADING_COMPENSATION_CONFIG;
import static frc.robot.Constants.DriveConstants.TELEOP_CONTROL_CONFIG;

import java.util.function.BooleanSupplier;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl;
import frc.robot.BreakerLib.util.BreakerLibVersion;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.BreakerLib.util.loging.BreakerLog.Metadata;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final BreakerXboxController controller = new BreakerXboxController(Constants.OperatorConstants.CONTROLLER_PORT);
  private final Drivetrain drivetrain = new Drivetrain();

  private BreakerInputStream driverX, driverY, driverOmega;
  public RobotContainer() {
    configureControls();
    BreakerLog.setOptions(new DogLogOptions(true, false, true, true, true, 2000)); 
    if (RobotBase.isReal()) {
      BreakerLog.setPdh(new PowerDistribution(1, ModuleType.kRev));
    }
    BreakerLog.addCANBus(GeneralConstants.DRIVE_CANIVORE_BUS);
    BreakerLog.setEnabled(true);
    BreakerLog.logMetadata(new Metadata("Brick", 2024, "Roman Abrahamson", GeneralConstants.GIT_INFO));
  }

  private void configureControls() {
    BreakerInputStream2d driverTranslation = controller.getLeftThumbstick();
    driverTranslation
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
            .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
    driverX = driverTranslation.getX();
    driverY = driverTranslation.getY();

    driverOmega = controller.getRightThumbstick().getX()
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.0, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));

    drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, TELEOP_CONTROL_CONFIG));

    

  
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
