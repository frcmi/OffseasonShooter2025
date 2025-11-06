package team5937.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team5937.frc2025.autoalign.AimToAngle;
import team5937.frc2025.autoalign.AimToReef;
import team5937.frc2025.autoalign.pointtopoint.PointToPoint;
import team5937.frc2025.subsystems.drive.Drive;
import team5937.lib.alliancecolor.AllianceColor;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private final double DEADBAND = 0.02;
    private final double FF_START_DELAY = 2.0; // Secs
    private final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private final Drive drive;
    private final PointToPoint pointToPoint;
    private final AimToReef aimToReef;
    private final AimToAngle aimToAngle;

    public DriveCommands(
            Drive drive, PointToPoint pointToPoint, AimToReef aimToReef, AimToAngle aimToAngle) {
        this.drive = drive;
        this.pointToPoint = pointToPoint;
        this.aimToReef = aimToReef;
        this.aimToAngle = aimToAngle;
    }

    public Command resetPose() {
        return Commands.runOnce(
                        () ->
                                drive.setPose(
                                        new Pose2d(
                                                drive.getPose().getTranslation(),
                                                AllianceColor.isAllianceBlue()
                                                        ? Rotation2d.kZero
                                                        : Rotation2d.k180deg)))
                .ignoringDisable(true);
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public Command drive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
        return Commands.run(
                () -> {
                    Translation2d linearVelocity = Translation2d.kZero;
                    double rot = 0;

                    if (pointToPoint.isRunning()) {
                        Optional<ChassisSpeeds> result = pointToPoint.getPointToPointSpeeds();
                        if (result.isPresent()) {
                            ChassisSpeeds speeds = result.get();
                            drive.runVelocity(speeds);

                            Logger.recordOutput("DriveCommands/TeleopDriveCommand/Speeds", speeds);
                            return;
                        }
                    }

                    linearVelocity =
                            getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    if (aimToReef.isRunning()) {
                        rot = aimToReef.getAimToReefVelocity();
                    } else if (aimToAngle.isRunning()) {
                        rot = aimToAngle.getAimToAngleVelocity();
                    } else {
                        rot = MathUtil.applyDeadband(rotSupplier.getAsDouble(), DEADBAND);
                        rot = Math.copySign(rot * rot, rot);
                    }

                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    rot * drive.getMaxAngularSpeedRadPerSec());
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    AllianceColor.isAllianceRed()
                                            ? drive.getRotation().plus(Rotation2d.kPi)
                                            : drive.getRotation()));

                    Logger.recordOutput("DriveCommands/TeleopDriveCommand/Speeds", speeds);
                },
                drive);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS =
                                            (sumY * sumX2 - sumX * sumXY)
                                                    / (n * sumX2 - sumX * sumX);
                                    double kV =
                                            (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println(
                                            "********** Drive FF Characterization Results"
                                                    + " **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                        () -> {
                                            var rotation = drive.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions =
                                                    drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta +=
                                                        Math.abs(positions[i] - state.positions[i])
                                                                / 4.0;
                                            }
                                            double wheelRadius =
                                                    (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                            / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization"
                                                            + " Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: "
                                                            + formatter.format(wheelDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: "
                                                            + formatter.format(state.gyroDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(
                                                                    Units.metersToInches(
                                                                            wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
