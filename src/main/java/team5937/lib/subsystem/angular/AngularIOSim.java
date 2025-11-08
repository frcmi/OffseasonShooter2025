package team5937.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kDt;
import static team5937.lib.subsystem.angular.AngularIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import team5937.lib.sim.CurrentDrawCalculatorSim;
import team5937.lib.sim.PivotSim;
import team5937.lib.subsystem.DeviceConnectedStatus;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Supplier;

public class AngularIOSim implements AngularIO {
    private final PivotSim pivot;
    private final ProfiledPIDController posController;
    private final ProfiledPIDController velController;

    private final AngularIOSimConfig deviceConfig;

    private final double[] motorTemperatures = new double[] {};
    private final DeviceConnectedStatus[] deviceConnectedStatuses = new DeviceConnectedStatus[] {};

    private AngularIOOutputMode outputMode = kNeutral;
    private Optional<Angle> goalPos = Optional.empty();
    private Optional<AngularVelocity> goalVel = Optional.empty();
    private Optional<Voltage> openLoopVolts = Optional.empty();

    private Current supplyCurrent = Amps.of(0.0);

    private Optional<Supplier<Rotation2d>> realAngleFromSubsystemAngleZero = Optional.empty();
    private Optional<Supplier<Distance>> armLength = Optional.empty();

    private AngularVelocity velocity = RadiansPerSecond.of(0.0);

    public AngularIOSim(
            AngularIOSimConfig config, CurrentDrawCalculatorSim currentDrawCalculatorSim) {
        this.deviceConfig = config;

        // Hardware
        DCMotor motor = config.getMotor();
        pivot =
                new PivotSim(
                        motor,
                        config.getMotorRotationsPerOutputRotations(),
                        config.getMoi().in(KilogramSquareMeters),
                        armLength.orElse(() -> Meters.of(0.0)).get().in(Meters),
                        config.getPhysicalMinAngle().in(Radians),
                        config.getPhysicalMaxAngle().in(Radians),
                        realAngleFromSubsystemAngleZero,
                        config.getResetAngle().in(Radians));
        posController =
                new ProfiledPIDController(
                        config.getKP(),
                        config.getKI(),
                        config.getKD(),
                        new TrapezoidProfile.Constraints(
                                config.getCruiseVelocity().in(RadiansPerSecond),
                                config.getAcceleration().in(RadiansPerSecondPerSecond)));
        velController = new ProfiledPIDController(
            config.getKP(), 
            config.getKI(), 
            config.getKD(), new TrapezoidProfile.Constraints(
                config.getAcceleration().in(RadiansPerSecondPerSecond),
                1e9
            ));

        currentDrawCalculatorSim.registerCurrentDraw(() -> supplyCurrent);
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
        armLength.ifPresent(length -> pivot.setArmLength(length.get()));

        Optional<Angle> posSet = Optional.empty();
        Optional<AngularVelocity> velSet = Optional.empty();
        switch (outputMode) {
            case kClosedLoop -> {
                    inputs.appliedVolts =
                            Volts.of(
                                    MathUtil.clamp(
                                            posController.calculate(
                                                            pivot.getAngleRads(),
                                                            goalPos.orElse(Radians.of(0.0))
                                                                    .in(Radians)) * RobotController.getBatteryVoltage(),
                                            -12.0,
                                            12.0));
                    posSet = Optional.of(Radians.of(posController.getSetpoint().position));
                    velSet = Optional.of(RadiansPerSecond.of(posController.getSetpoint().velocity));
                }
            case kOpenLoop ->
                    inputs.appliedVolts =
                            Volts.of(MathUtil.clamp(openLoopVolts.orElse(Volts.of(0.0)).in(Volts), -12.0, 12.0));
            case kVelocity -> {
                    double goalVelValue = goalVel.orElse(RadiansPerSecond.of(0.0)).in(RadiansPerSecond);
                    inputs.appliedVolts = Volts.of(MathUtil.clamp(
                    velController.calculate(
                        pivot.getVelocityRadPerSec(), 
                        goalVelValue
                    ) + velController.getSetpoint().position * deviceConfig.getKV(), -12.0, 12.0));
                    velSet = Optional.of(RadiansPerSecond.of(velController.getSetpoint().position));
            }
            case kNeutral -> inputs.appliedVolts = Volts.of(0.0);
        }
        pivot.setInput(inputs.appliedVolts.in(Volts));
        pivot.update(kDt);

        inputs.goalPos = posSet.orElse(Radians.of(0.0));
        inputs.goalVel = velSet.orElse(RadiansPerSecond.of(0.0));
        
        inputs.angle = Radians.of(pivot.getAngleRads());

        inputs.supplyCurrent = Amps.of(pivot.getCurrentDrawAmps() * inputs.appliedVolts.abs(Volts)/RobotController.getBatteryVoltage());
        inputs.statorCurrent = Amps.of(pivot.getCurrentDrawAmps());
        this.supplyCurrent = inputs.supplyCurrent;

        inputs.velocity = RadiansPerSecond.of(pivot.getVelocityRadPerSec());
        this.velocity = inputs.velocity;
        inputs.acceleration = RadiansPerSecondPerSecond.of(0.0);

        inputs.motorTemperatures = this.motorTemperatures;
        inputs.deviceConnectedStatuses = this.deviceConnectedStatuses;

        inputs.neutralMode = deviceConfig.getNeutralMode();
        inputs.IOOutputMode = this.outputMode;
    }

    @Override
    public void setAngle(Angle angle) {
        this.goalPos = Optional.of(angle);
        this.goalVel = Optional.empty();
        this.openLoopVolts = Optional.empty();

        posController.reset(pivot.getAngleRads(), velocity.in(RadiansPerSecond));
        outputMode = kClosedLoop;
    }

    @Override
    public void setVelocity(AngularVelocity angVel) {
        this.goalPos = Optional.empty();
        this.goalVel = Optional.of(angVel);
        this.openLoopVolts = Optional.empty();
        velController.reset(this.velocity.in(RadiansPerSecond));
        outputMode = kVelocity;
    }

    @Override
    public void setOpenLoop(Voltage openLoopVoltage) {
        this.goalPos = Optional.empty();
        this.goalVel = Optional.empty();
        this.openLoopVolts = Optional.of(openLoopVoltage);
        outputMode = kOpenLoop;
    }

    @Override
    public void stop() {
        setOpenLoop(Volts.of(0.0));
        outputMode = kNeutral;
    }

    @Override
    public void setPIDV(double kP, double kI, double kD, double kV) {
        deviceConfig.setKP(kP);
        deviceConfig.setKI(kI);
        deviceConfig.setKD(kD);
        deviceConfig.setKV(kV);

        posController.setPID(kP, kI, kD);
        velController.setPID(kP, kI, kD);
    }

    @Override
    public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
        deviceConfig.setCruiseVelocity(cruiseVelocity);
        deviceConfig.setAcceleration(acceleration);

        posController.setConstraints(
                new TrapezoidProfile.Constraints(
                        cruiseVelocity.in(RadiansPerSecond),
                        acceleration.in(RadiansPerSecondPerSecond)));
        velController.setConstraints(
                new TrapezoidProfile.Constraints(
                        acceleration.in(RadiansPerSecondPerSecond),
                        1e9));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        deviceConfig.setNeutralMode(neutralMode);
    }

    public void setRealAngleFromSubsystemAngleZeroSupplier(
            Supplier<Rotation2d> realAngleFromSubsystemAngleZero) {
        this.realAngleFromSubsystemAngleZero = Optional.ofNullable(realAngleFromSubsystemAngleZero);
    }

    public void setArmLengthSupplier(Supplier<Distance> length) {
        this.armLength = Optional.of(length);
    }
}
