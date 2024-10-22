// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.HEADING_COMPENSATION_CONFIG;

import java.io.InputStream;
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
    BreakerLog.setEnabled(true);
    BreakerLog.logMetadata(new Metadata("Brick", 2024, "Roman Abrahamson", GeneralConstants.GIT_INFO));
  }

  private void configureControls() {
    driverX = controller.getLeftThumbstick().getStreamX();
    driverY = controller.getLeftThumbstick().getStreamY();
    BreakerInputStream translationalMag =
        BreakerInputStream.hypot(driverX, driverY)
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));

    BreakerInputStream translationalTheta = BreakerInputStream.atan(driverX, driverY);

    driverX = translationalMag.scale(translationalTheta.map(Math::cos));
    driverY = translationalMag.scale(translationalTheta.map(Math::sin));

    driverOmega = controller.getRightThumbstick().getStreamX()
            .clamp(1.0)
            .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.0, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));

    drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, HEADING_COMPENSATION_CONFIG));

    

  
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
