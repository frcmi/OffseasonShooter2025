// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.constants.Intake;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

import java.util.function.Supplier;

import team5937.frc2025.subsystems.intake.IntakeState;
import team5937.lib.sim.PivotSim;
import team5937.lib.subsystem.angular.AngularIOSimConfig;
import team5937.lib.subsystem.angular.AngularIOTalonFXConfig;
import team5937.lib.subsystem.angular.AngularSubsystemConfig;

public class PivotConstants {
    public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
            () -> Rotation2d.kZero;

    public static final AngularSubsystemConfig kSubsystemConfigReal =
            AngularSubsystemConfig.builder()
                    .logKey("Pivot")
                    .bus(kRioBus)
                    .positionTolerance(Degrees.of(2.0))
                    .velocityTolerance(DegreesPerSecond.of(4.58))
                    .kP(5.0)
                    .kI(0.0)
                    .kD(0.0)
                    .cruiseVelocity(DegreesPerSecond.of(2000.0))
                    .acceleration(DegreesPerSecondPerSecond.of(4500.0))
                    .build();

    public static final AngularIOTalonFXConfig kTalonFXConfig =
            AngularIOTalonFXConfig.builder()
                    .masterId(32)
                    .bus(kRioBus)
                    .resetAngle(IntakeState.kStowed.getPivot())
                    .softMinAngle(IntakeState.kIntaking.getPivot())
                    .softMaxAngle(IntakeState.kStowed.getPivot())
                    .motorRotationsPerOutputRotations((38.0/12.0) * (56.0/14.0) * (24.0/18.0) * (36.0/12.0)) // The reductions on the intake
                    .outputAnglePerOutputRotation(Rotations.of(1.0))
                    .inverted(InvertedValue.Clockwise_Positive)
                    .supplyCurrentLimit(Amps.of(30.0))
                    .statorCurrentLimit(Amps.of(90.0))
                    .neutralMode(NeutralModeValue.Brake)
                    .kP(kSubsystemConfigReal.getKP())
                    .kI(kSubsystemConfigReal.getKI())
                    .kD(kSubsystemConfigReal.getKD())
                    .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
                    .acceleration(kSubsystemConfigReal.getAcceleration())
                    .build();

    public static final AngularSubsystemConfig kSubsystemConfigSim =
            AngularSubsystemConfig.builder()
                    .logKey(kSubsystemConfigReal.getLogKey())
                    .bus(kSubsystemConfigReal.getBus())
                    .positionTolerance(kSubsystemConfigReal.getPositionTolerance())
                    .velocityTolerance(kSubsystemConfigReal.getVelocityTolerance())
                    .kP(5.0)
                    .kI(0.0)
                    .kD(0.0)
                    .cruiseVelocity(kSubsystemConfigReal.getCruiseVelocity())
                    .acceleration(kSubsystemConfigReal.getAcceleration())
                    .build();

    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(293.783602 * 0.000292639653); // Converted from lb in^2 to kg m^2
    public static final AngularIOSimConfig kSimConfig =
            AngularIOSimConfig.builder()
                    .motor(DCMotor.getKrakenX60(1))
                    .moi(kMOI)
                    .resetAngle(kTalonFXConfig.getResetAngle())
                    .physicalMinAngle(IntakeState.kIntaking.getPivot())
                    .physicalMaxAngle(IntakeState.kStowed.getPivot())
                    .motorRotationsPerOutputRotations(
                            kTalonFXConfig.getMotorRotationsPerOutputRotations())
                    .neutralMode(kTalonFXConfig.getNeutralMode())
                    .kP(kSubsystemConfigSim.getKP())
                    .kI(kSubsystemConfigSim.getKI())
                    .kD(kSubsystemConfigSim.getKD())
                    .cruiseVelocity(kSubsystemConfigSim.getCruiseVelocity())
                    .acceleration(kSubsystemConfigSim.getAcceleration())
                    .build();
}

