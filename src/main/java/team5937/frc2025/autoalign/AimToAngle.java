package team5937.frc2025.autoalign;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import team5937.frc2025.subsystems.drive.Drive;
import team5937.lib.alliancecolor.AllianceUpdatedObserver;
import team5937.lib.subsystem.VirtualSubsystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AimToAngle extends VirtualSubsystem implements AllianceUpdatedObserver {
    private final ProfiledPIDController rotationController =
            new ProfiledPIDController(2.0, 0, 0, new Constraints(25 * Math.PI, 15 * Math.PI));

    private final State rotationGoal = new State(0, 0);
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private Alliance alliance = Alliance.Blue;

    @Getter @AutoLogOutput private boolean running = false;
    public Trigger isRunning = new Trigger(this::isRunning);

    public AimToAngle(Drive drive) {
        this.pose = drive::getPose;
        this.chassisSpeeds = drive::getRobotRelativeChassisSpeeds;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        warmupLogs();
    }

    public Command run(Supplier<Optional<Double>> rot) {
        return parallel(startEnd(this::start, this::end), setRotation(rot));
    }

    private void start() {
        this.rotationController.reset(
                pose.get().getRotation().getRadians(), chassisSpeeds.get().omegaRadiansPerSecond);
        this.running = true;
    }

    private void end() {
        this.running = false;
        this.rotationController.reset(
                pose.get().getRotation().getRadians(), chassisSpeeds.get().omegaRadiansPerSecond);
    }

    public double getAimToAngleVelocity() {
        final double wantedVelocityRads =
                rotationController.calculate(pose.get().getRotation().getRadians(), rotationGoal);
        Logger.recordOutput("AimToAngle/WantedVelocityRads", wantedVelocityRads);

        return wantedVelocityRads;
    }

    @Override
    public void onAllianceFound(Alliance alliance) {
        this.alliance = alliance;
    }

    private Command setRotation(Supplier<Optional<Double>> rot) {
        return Commands.run(
                () -> {
                    if (rot.get().isEmpty()) return;
                    rotationGoal.position =
                            (alliance.equals(Alliance.Blue) ? 0 : Math.PI) + rot.get().get();
                });
    }

    private void warmupLogs() {
        start();
        getAimToAngleVelocity();
        end();
    }
}
