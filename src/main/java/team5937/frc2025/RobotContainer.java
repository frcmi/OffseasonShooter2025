package team5937.frc2025;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.subsystems.vision.VisionConstants.camera0Name;
import static team5937.frc2025.subsystems.vision.VisionConstants.camera1Name;
import static team5937.frc2025.subsystems.vision.VisionConstants.robotToCamera0;
import static team5937.frc2025.subsystems.vision.VisionConstants.robotToCamera1;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team5937.frc2025.autoalign.AimToAngle;
import team5937.frc2025.autoalign.AimToReef;
import team5937.frc2025.autoalign.AlignPose;
import team5937.frc2025.autoalign.pointtopoint.PointToPoint;
import team5937.frc2025.autoalign.pointtopoint.PointToPointReef;
import team5937.frc2025.commands.DriveCommands;
import team5937.frc2025.commands.RobotSuperstructure;
import team5937.frc2025.constants.ModeConstants;
import team5937.frc2025.constants.RobotConstants;
import team5937.frc2025.constants.TunerConstants;
import team5937.frc2025.constants.VisionConstants;
import team5937.frc2025.constants.Intake.PivotConstants;
import team5937.frc2025.constants.Intake.RollerConstants;
import team5937.frc2025.subsystems.drive.*;
import team5937.frc2025.subsystems.intake.Intake;
import team5937.frc2025.subsystems.vision.Vision;
import team5937.frc2025.subsystems.vision.VisionIO;
import team5937.frc2025.subsystems.vision.VisionIOLimelight;
import team5937.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import team5937.lib.alliancecolor.AllianceChecker;
import team5937.lib.controller.Joysticks;
import team5937.lib.sim.CurrentDrawCalculatorSim;
import team5937.lib.sim.CustomDCMotor;
import team5937.lib.subsystem.VirtualSubsystem;
import team5937.lib.subsystem.angular.AngularIO;
import team5937.lib.subsystem.angular.AngularIOSim;
import team5937.lib.subsystem.angular.AngularIOSimConfig;
import team5937.lib.subsystem.angular.AngularIOTalonFX;
import team5937.lib.subsystem.angular.AngularSubsystem;
import team5937.lib.subsystem.angular.AngularSubsystemConfig;
import team5937.lib.subsystem.linear.LinearIOSim;
import team5937.lib.subsystem.linear.LinearIOTalonFX;
import team5937.lib.subsystem.linear.LinearSubsystem;

import java.util.Optional;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends VirtualSubsystem {
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;

    private final RobotSuperstructure superstructure;


    private final PointToPoint pointToPoint;
    private final PointToPointReef pointToPointReef;

    @SuppressWarnings("FieldCanBeLocal")
    private final AimToReef aimToReef;

    private final AimToAngle aimToAngle;

    private final DriveCommands driveCommands;

    @SuppressWarnings("FieldCanBeLocal")
    private final AllianceChecker allianceChecker = new AllianceChecker();

    @SuppressWarnings("FieldCanBeLocal")
    private final CurrentDrawCalculatorSim currentDrawCalculatorSim =
            new CurrentDrawCalculatorSim();

    private final Joysticks controller = new Joysticks(0);

    private final LoggedDashboardChooser<Command> autoChooser;
    private final Field2d field = new Field2d();

    private final Alert autoAlert = new Alert("No auto selected!", Alert.AlertType.kWarning);
    private final Alert controllerOneAlert =
            new Alert("Controller 1 is unplugged!", Alert.AlertType.kWarning);

    public RobotContainer() {
        switch (ModeConstants.kCurrentMode) {
            case kReal:
                // Real robot, instantiate hardware IO implementations
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(camera0Name, drive::getRotation),
                        new VisionIOLimelight(camera1Name, drive::getRotation)
                );
                intake = new Intake(
                        new AngularSubsystem(
                                new AngularIOTalonFX(RollerConstants.kTalonFXConfig), RollerConstants.kSubsystemConfigReal), 
                        new AngularSubsystem(
                                new AngularIOTalonFX(PivotConstants.kTalonFXConfig), PivotConstants.kSubsystemConfigReal)
                );
                break;

            case kSim:
                // Sim robot, instantiate physics sim IO implementations
                drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft, currentDrawCalculatorSim),
                        new ModuleIOSim(TunerConstants.FrontRight, currentDrawCalculatorSim),
                        new ModuleIOSim(TunerConstants.BackLeft, currentDrawCalculatorSim),
                        new ModuleIOSim(TunerConstants.BackRight, currentDrawCalculatorSim));
                vision =
                        new Vision(
                                drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

                AngularIOSim pivotIO = new AngularIOSim(PivotConstants.kSimConfig, currentDrawCalculatorSim);
                pivotIO.setRealAngleFromSubsystemAngleZeroSupplier(PivotConstants.kRealAngleFromSubsystemAngleZeroSupplier);
                intake = new Intake(
                        new AngularSubsystem(
                                new AngularIOSim(RollerConstants.kSimConfig, currentDrawCalculatorSim), RollerConstants.kSubsystemConfigSim), 
                        new AngularSubsystem(
                                pivotIO, PivotConstants.kSubsystemConfigReal)
                );
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive();
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                intake = new Intake();
                break;
        }

        superstructure = new RobotSuperstructure(intake);

        pointToPoint = new PointToPoint(drive, () -> 0.0, field);
        pointToPointReef = new PointToPointReef(pointToPoint, drive);
        aimToReef = new AimToReef(drive);
        aimToAngle = new AimToAngle(drive);
        driveCommands = new DriveCommands(drive, pointToPoint, aimToReef, aimToAngle);

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                driveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                driveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        allianceChecker.registerObservers(aimToReef, aimToAngle);

        logInit();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(
                driveCommands.drive(
                        controller::getLeftStickY,
                        () -> -controller.getLeftStickX(),
                        () -> -controller.getRightStickX()));

        controller.rightBumper.onTrue(superstructure.intakeDeploy());
        controller.rightBumper.onFalse(superstructure.intakeStowed());
    }

    @Override
    public void periodic() {
        logPeriodic();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void enableInit() {

    }

    private void logInit() {
        SmartDashboard.putData("Field", field);

        Logger.recordOutput(
                "Poses/AprilTagField",
                VisionConstants.kAprilTagField.values().toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Poses/WeldedAprilTagField",
                VisionConstants.kWeldedAprilTagField.values().toArray(new Pose3d[0]));
        Logger.recordOutput(
                "Poses/AndyMarkAprilTagField",
                VisionConstants.kAndyMarkAprilTagField.values().toArray(new Pose3d[0]));

        Logger.recordOutput(
                "Poses/BlueCoralPoses",
                AlignPose.Blue.Coral.posesList.stream()
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));
        Logger.recordOutput(
                "Poses/RedCoralPoses",
                AlignPose.Red.Coral.posesList.stream()
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));
        Logger.recordOutput(
                "Poses/BlueAlgaePoses",
                AlignPose.Blue.Algae.poseList.stream()
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));
        Logger.recordOutput(
                "Poses/RedAlgaePoses",
                AlignPose.Red.Algae.poseList.stream()
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));

        Logger.recordOutput(
                "Poses/AllAutoAlignPoses",
                Stream.of(
                                AlignPose.Blue.Coral.posesList.stream(),
                                AlignPose.Red.Coral.posesList.stream(),
                                AlignPose.Blue.Algae.poseList.stream(),
                                AlignPose.Red.Algae.poseList.stream())
                        .flatMap(s -> s)
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));

        Logger.recordOutput(
                "Poses/AlgLevelPoses",
                AlignPose.Red.Algae.poseList.stream()
                        .map(AlignPose::pose)
                        .distinct()
                        .toArray(Pose2d[]::new));
    }

    private void logPeriodic() {
        boolean disabled = DriverStation.isDisabled();

        autoAlert.set(autoChooser.getSendableChooser().getSelected().equals("None") && disabled);
        controllerOneAlert.set(!controller.getController().isConnected());

        field.setRobotPose(drive.getPose());
    }
}
