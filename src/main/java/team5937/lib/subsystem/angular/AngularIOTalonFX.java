package team5937.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kMaxTimeoutMS;
import static team5937.lib.subsystem.angular.AngularIOOutputMode.*;
import static team5937.lib.utils.PhoenixUtils.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import team5937.lib.subsystem.DeviceConnectedStatus;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

public class AngularIOTalonFX implements AngularIO {
    // Hardware
    private final TalonFX master;
    private final List<TalonFX> followers;

    // Config
    private final TalonFXConfiguration masterConfig;
    private final TalonFXConfiguration followerConfig;

    // Status Signals
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final List<StatusSignal<Temperature>> motorTemperatures;

    private final MotionMagicVoltage motionMagicPos; 
    private final MotionMagicVelocityVoltage motionMagicVel; 
    private final VoltageOut voltageOut;

    private final AngularIOTalonFXConfig deviceConfig;

    private final Alert configurationsNotAppliedAlert =
            new Alert("Configurations for AngularSubsystem not applied!", Alert.AlertType.kError);

    private AngularIOOutputMode outputMode = kNeutral;
    private Optional<Angle> goalPos = Optional.empty();
    private Optional<AngularVelocity> goalVel = Optional.empty();

    public AngularIOTalonFX(AngularIOTalonFXConfig config) {
        this.deviceConfig = config;

        master = new TalonFX(config.getMasterId(), config.getBus());
        followers =
                config.getFollowerIds().stream()
                        .map(id -> new TalonFX(id, config.getBus()))
                        .toList();

        // Clear sticky faults.
        master.clearStickyFaults(kMaxTimeoutMS);
        followers.forEach(talonFX -> talonFX.clearStickyFaults(kMaxTimeoutMS));

        // Set follower control.
        followers.forEach(
                talonFX ->
                        talonFX.setControl(
                                new Follower(config.getMasterId(), config.isOpposeMaster())));

        // Apply motor configs.
        masterConfig = getMasterConfig();
        followerConfig = getMasterConfig();
        followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        AtomicBoolean applySuccess =
                new AtomicBoolean(
                        tryUntilOk(
                                () -> master.getConfigurator().apply(masterConfig, kMaxTimeoutMS)));
        followers.forEach(
                talonFX ->
                        applySuccess.set(
                                applySuccess.get()
                                        & tryUntilOk(
                                                () ->
                                                        talonFX.getConfigurator()
                                                                .apply(
                                                                        followerConfig,
                                                                        kMaxTimeoutMS))));
        configurationsNotAppliedAlert.set(!applySuccess.get());

        motionMagicPos =
                new MotionMagicVoltage(
                        Rotations.of(
                                config.getResetAngle().in(Radians)
                                        / config.getOutputAnglePerOutputRotation().in(Radians)));
        motionMagicVel = new MotionMagicVelocityVoltage(RotationsPerSecond.of(0.0));
        voltageOut = new VoltageOut(0.0);

        // Set signals.
        position = master.getPosition();
        appliedVolts = master.getMotorVoltage();
        statorCurrent = master.getStatorCurrent();
        supplyCurrent = master.getSupplyCurrent();
        velocity = master.getVelocity();
        acceleration = master.getAcceleration();
        motorTemperatures = new ArrayList<>();
        motorTemperatures.add(master.getDeviceTemp());
        motorTemperatures.addAll(followers.stream().map(CoreTalonFX::getDeviceTemp).toList());
    }

    private TalonFXConfiguration getMasterConfig() {
        final TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Slot0.kP =
                deviceConfig.getKP() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.Slot0.kI =
                deviceConfig.getKI() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.Slot0.kD =
                deviceConfig.getKD() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.Slot0.kV =
                deviceConfig.getKV() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);

        configuration.MotionMagic.MotionMagicCruiseVelocity =
                deviceConfig.getCruiseVelocity().in(RadiansPerSecond)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.MotionMagic.MotionMagicAcceleration =
                deviceConfig.getAcceleration().in(RadiansPerSecondPerSecond)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);

        configuration.CurrentLimits.SupplyCurrentLimit =
                deviceConfig.getSupplyCurrentLimit().in(Amps);
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit =
                deviceConfig.getStatorCurrentLimit().in(Amps);
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.Feedback.SensorToMechanismRatio =
                deviceConfig.getMotorRotationsPerOutputRotations();

        configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                deviceConfig.getSoftMaxAngle().in(Radians)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable =
                deviceConfig.getSoftMaxAngle().gte(Radians.of(Double.POSITIVE_INFINITY));
        configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                deviceConfig.getSoftMinAngle().in(Radians)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable =
                deviceConfig.getSoftMinAngle().lte(Radians.of(Double.NEGATIVE_INFINITY));

        configuration.MotorOutput.NeutralMode = deviceConfig.getNeutralMode();
        configuration.MotorOutput.Inverted = deviceConfig.getInverted();
        return configuration;
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                position, velocity, acceleration, statorCurrent, appliedVolts, supplyCurrent);
        motorTemperatures.forEach(StatusSignal::refresh);

        inputs.angle =
                Radians.of(
                        position.getValueAsDouble()
                                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
        inputs.appliedVolts = appliedVolts.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.velocity =
                RadiansPerSecond.of(
                        velocity.getValueAsDouble()
                                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
        inputs.acceleration =
                RadiansPerSecondPerSecond.of(
                        acceleration.getValueAsDouble()
                                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));

        inputs.motorTemperatures =
                motorTemperatures.stream()
                        .mapToDouble(signal -> signal.getValue().in(Celsius))
                        .toArray();

        int size = followers.size() + 1;
        if (inputs.deviceConnectedStatuses.length != size)
            inputs.deviceConnectedStatuses = new DeviceConnectedStatus[size];
        if (inputs.deviceConnectedStatuses[0] == null) {
            inputs.deviceConnectedStatuses[0] =
                    new DeviceConnectedStatus(
                            BaseStatusSignal.isAllGood(
                                    position,
                                    appliedVolts,
                                    supplyCurrent,
                                    statorCurrent,
                                    acceleration,
                                    motorTemperatures.get(0)),
                            deviceConfig.getMasterId());
        } else {
            inputs.deviceConnectedStatuses[0].setConnected(
                    BaseStatusSignal.isAllGood(
                            position,
                            appliedVolts,
                            supplyCurrent,
                            statorCurrent,
                            acceleration,
                            motorTemperatures.get(0)));
        }

        for (int i = 0; i < followers.size(); i++) {
            if (inputs.deviceConnectedStatuses[i + 1] == null) {
                inputs.deviceConnectedStatuses[i + 1] =
                        new DeviceConnectedStatus(
                                BaseStatusSignal.isAllGood(motorTemperatures.get(i + 1)),
                                deviceConfig.getFollowerIds().get(i));
            } else {
                inputs.deviceConnectedStatuses[i + 1].setConnected(
                        BaseStatusSignal.isAllGood(motorTemperatures.get(i + 1)));
            }
        }

        inputs.neutralMode = deviceConfig.getNeutralMode();

        inputs.IOOutputMode = this.outputMode;
        inputs.goalPos = this.goalPos.orElse(Radians.of(0.0));
        inputs.goalVel = this.goalVel.orElse(RadiansPerSecond.of(0.0));
    }

    @Override
    public void setAngle(Angle angle) {
        master.setControl(
                motionMagicPos.withPosition(
                        angle.in(Radians)
                                / deviceConfig.getOutputAnglePerOutputRotation().in(Radians)));
        goalPos = Optional.of(angle);
        goalVel = Optional.empty();
        outputMode = kClosedLoop;
    }

    @Override
    public void setVelocity(AngularVelocity angVel) {
        master.setControl(
                motionMagicVel.withVelocity(
                        angVel.in(RadiansPerSecond)
                                / deviceConfig.getOutputAnglePerOutputRotation().in(Radians)));
        goalPos = Optional.empty();
        goalVel = Optional.of(angVel);
        outputMode = kVelocity;
    }

    @Override
    public void setOpenLoop(Voltage voltage) {
        master.setControl(voltageOut.withOutput(voltage));
        goalPos = Optional.empty();
        goalVel = Optional.empty();
        outputMode = kOpenLoop;
    }

    @Override
    public void stop() {
        master.stopMotor();
        outputMode = kNeutral;
    }

    @Override
    public void resetAngle() {
        resetAngle(deviceConfig.getResetAngle());
    }

    @Override
    public void resetAngle(Angle angle) {
        master.setPosition(
                Rotations.of(
                        angle.in(Radians)
                                / deviceConfig.getOutputAnglePerOutputRotation().in(Radians)));
    }

    @Override
    public void setPIDV(double kP, double kI, double kD, double kV) {
        deviceConfig.setKP(kP);
        deviceConfig.setKI(kI);
        deviceConfig.setKD(kD);
        deviceConfig.setKV(kV);

        masterConfig.Slot0.kP = kP * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        masterConfig.Slot0.kI = kI * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        masterConfig.Slot0.kD = kD * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        masterConfig.Slot0.kV = kV * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        master.getConfigurator().apply(masterConfig, 0.0);
    }

    @Override
    public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
        deviceConfig.setCruiseVelocity(cruiseVelocity);
        deviceConfig.setAcceleration(acceleration);

        masterConfig.MotionMagic.MotionMagicCruiseVelocity =
                cruiseVelocity.in(RadiansPerSecond)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        masterConfig.MotionMagic.MotionMagicAcceleration =
                acceleration.in(RadiansPerSecondPerSecond)
                        / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
        master.getConfigurator().apply(masterConfig, 0.0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        deviceConfig.setNeutralMode(neutralMode);
        master.setNeutralMode(neutralMode, 0.0);
        followers.forEach(talonFX -> talonFX.setNeutralMode(neutralMode, 0.0));
    }

    @Override
    public void setLogKey(String logKey) {
        configurationsNotAppliedAlert.setText(
                String.format("Configurations for TalonFX %s not applied!", logKey));
    }
}
