package team5937.frc2025.autoalign;

import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import team5937.frc2025.subsystems.drive.Drive;
import team5937.lib.alliancecolor.AllianceUpdatedObserver;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AimToReef implements AllianceUpdatedObserver {
    private static final Pose2d kCenterOfRedReef =
            new Pose2d(new Translation2d(13.07, 4.03), Rotation2d.kZero);
    private static final Pose2d kCenterOfBlueReef =
            new Pose2d(new Translation2d(4.49, 4.03), Rotation2d.kZero);

    private final ProfiledPIDController rotationController =
            new ProfiledPIDController(4.5, 0, 0, new Constraints(25 * Math.PI, 15 * Math.PI));

    private Pose2d fieldToReef = new Pose2d();
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    @Getter @AutoLogOutput private boolean running = false;
    public Trigger isRunning = new Trigger(this::isRunning);

    public AimToReef(Drive drive) {
        this.pose = drive::getPose;
        this.chassisSpeeds = drive::getRobotRelativeChassisSpeeds;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        warmupLogs();
    }

    public Command run() {
        return startEnd(this::start, this::end);
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

    public double getAimToReefVelocity() {
        final Pose2d fieldToRobot = this.pose.get();
        final ChassisSpeeds robotRelativeSpeeeds = this.chassisSpeeds.get();
        final Pose2d robotToReef = fieldToReef.relativeTo(fieldToRobot);
        final Transform2d robotToReefTransform = fieldToReef.minus(fieldToRobot);
        final Rotation2d robotToReefAngle =
                (robotToReef.getX() == 0.0 && robotToReef.getY() == 0.0)
                        ? Rotation2d.kZero
                        : new Rotation2d(robotToReef.getX(), robotToReef.getY());
        final double radiusToTarget = robotToReefTransform.getTranslation().getNorm();
        final double angularVelocityAroundReef =
                (robotToReefTransform.getRotation().getSin()
                                        * robotRelativeSpeeeds.vxMetersPerSecond
                                + robotToReefTransform.getRotation().getCos()
                                        * robotRelativeSpeeeds.vyMetersPerSecond)
                        / radiusToTarget;

        final double positionSetpointRads =
                fieldToRobot.getRotation().getRadians() + robotToReefAngle.getRadians();

        final double wantedVelocityRads =
                rotationController.calculate(
                        fieldToRobot.getRotation().getRadians(),
                        new State(positionSetpointRads, -angularVelocityAroundReef));
        Logger.recordOutput("AimToReef/WantedVelocityRads", wantedVelocityRads);
        Logger.recordOutput("AimToReef/AngularVelocityAroundReef", angularVelocityAroundReef);

        return wantedVelocityRads;
    }

    @Override
    public void onAllianceFound(Alliance alliance) {
        if (Alliance.Red.equals(alliance)) {
            this.fieldToReef = kCenterOfRedReef;
        } else {
            this.fieldToReef = kCenterOfBlueReef;
        }
        Logger.recordOutput("AimToReef/WantedReefPose", this.fieldToReef);
    }

    @AutoLogOutput
    public double distanceToReef() {
        return fieldToReef.getTranslation().getDistance(pose.get().getTranslation());
    }

    private void warmupLogs() {
        start();
        getAimToReefVelocity();
        end();
    }
}
