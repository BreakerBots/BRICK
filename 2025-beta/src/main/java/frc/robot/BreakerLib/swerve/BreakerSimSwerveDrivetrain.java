// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SimplifiedSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.swerve.BreakerSimSwerveDrivetrain.BreakerSwerveModuleSim;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.MapleSimConfig;

/** Add your docs here. */
public class BreakerSimSwerveDrivetrain {
    private final BreakerSwerveDriveSimulationBase driveSim;
    public BreakerSimSwerveDrivetrain (BreakerSwerveDrivetrain drivetrain, BreakerSwerveDrivetrainConstants constants, SwerveModuleConstants... swerveModuleConstants) {
        Supplier<GyroSimulation> gyroSim = GyroSimulation.getPigeon2();
        Supplier<SwerveModuleSimulation> swerveModuleSim = () -> new SwerveModuleSimulation(
            constants.simulationConfig.driveMotor, 
            constants.simulationConfig.driveMotor, 
            constants.simulationConfig.moduleConstants.SlipCurrent, 
            constants.simulationConfig.moduleConstants.DriveMotorGearRatio, 
            constants.simulationConfig.moduleConstants.SteerMotorGearRatio, 
            constants.simulationConfig.moduleConstants.DriveFrictionVoltage, 
            constants.simulationConfig.moduleConstants.SteerFrictionVoltage, 
            constants.simulationConfig.tireCoefficientOfFriction, 
            constants.simulationConfig.moduleConstants.WheelRadius, 
            constants.simulationConfig.moduleConstants.SteerInertia);

            driveSim = new BreakerSwerveDriveSimulationBase(
            new DriveTrainSimulationConfig(
                constants.simulationConfig.robotMass.in(Units.Kilograms), 
                constants.simulationConfig.bumperLengthX.in(Units.Meters), 
                constants.simulationConfig.bumperWidthY.in(Units.Meters), 
                Math.abs(constants.simulationConfig.moduleConstants.LocationX * 2), 
                Math.abs(constants.simulationConfig.moduleConstants.LocationY * 2), 
                swerveModuleSim, 
                gyroSim),
            new Pose2d(),
            drivetrain.getModules(),
            swerveModuleConstants
            );
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
    }

    private static class BreakerSwerveDriveSimulationBase extends SwerveDriveSimulation {
        private BreakerSwerveModuleSim[] moduleSims;
        public BreakerSwerveDriveSimulationBase(DriveTrainSimulationConfig config, Pose2d initialPoseOnField, SwerveModule[] modules, SwerveModuleConstants[] moduleConstants) {
            super(config, initialPoseOnField);
            moduleSims = new BreakerSwerveModuleSim[config.moduleTranslations.length];
            SwerveModuleSimulation[] mapleModuleSims = getModules();
            for (int i = 0; i < config.moduleTranslations.length; i++) {
                moduleSims[i] = new BreakerSwerveModuleSim(mapleModuleSims[i], modules[i], moduleConstants[i]);
            }
        }

        @Override
        public void simulationSubTick() {
            double supplyVoltage = RobotController.getBatteryVoltage();
            for (BreakerSwerveModuleSim moduleSim: moduleSims) {
                moduleSim.updateModel(supplyVoltage);
            }
            super.simulationSubTick();
            for (BreakerSwerveModuleSim moduleSim: moduleSims) {
                moduleSim.updateHardware();
            }
        }    
    }

    public static class BreakerSwerveModuleSim {
        private SwerveModuleSimulation moduleSim;
        private SwerveModule module;
        private SwerveModuleConstants constants;
        private TalonFXSimState driveSim = module.getDriveMotor().getSimState();
        private TalonFXSimState steerSim = module.getSteerMotor().getSimState();
        private CANcoderSimState encoderSim = module.getCANcoder().getSimState();

        public BreakerSwerveModuleSim(SwerveModuleSimulation moduleSim, SwerveModule module, SwerveModuleConstants constants) {
            this.moduleSim = moduleSim;
            this.module = module;
            this.constants = constants;
        }
        

        public void updateModel(double supplyVoltage) {
            driveSim.Orientation = constants.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            steerSim.Orientation = constants.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            
            driveSim.setSupplyVoltage(supplyVoltage);
            steerSim.setSupplyVoltage(supplyVoltage);
            encoderSim.setSupplyVoltage(supplyVoltage);

            moduleSim.requestDriveVoltageOut(driveSim.getMotorVoltage());
            moduleSim.requestSteerVoltageOut(steerSim.getMotorVoltage());
        }

        public void updateHardware() {
            driveSim.setRawRotorPosition(moduleSim.getDriveEncoderUnGearedPositionRad() / (2 * Math.PI));
            driveSim.setRotorVelocity(moduleSim.getDriveEncoderUnGearedSpeedRadPerSec() / (2 * Math.PI));
            
            steerSim.setRawRotorPosition(moduleSim.getSteerRelativeEncoderPositionRad() / (2 * Math.PI));
            steerSim.setRotorVelocity(moduleSim.getSteerRelativeEncoderSpeedRadPerSec() / (2 * Math.PI));
        }
    }

}
