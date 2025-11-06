package team5937.frc2025.autoalign.pointtopoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import team5937.frc2025.autoalign.AlignPose;
import team5937.frc2025.subsystems.drive.Drive;
import team5937.lib.alliancecolor.AllianceColor;

import java.util.List;
import java.util.function.Supplier;

public class PointToPointReef {
    private final PointToPoint pointToPoint;

    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    public PointToPointReef(PointToPoint pointToPoint, Drive drive) {
        this.pointToPoint = pointToPoint;
        this.pose = drive::getPose;
        this.chassisSpeeds = drive::getRobotRelativeChassisSpeeds;
    }

    public Command runClosestCoralPointToPoint() {
        return run(
                () ->
                        AllianceColor.isAllianceBlue()
                                ? AlignPose.Blue.Coral.posesList
                                : AlignPose.Red.Coral.posesList);
    }

    public Command runClosestAlgaePointToPoint() {
        return run(
                () ->
                        AllianceColor.isAllianceBlue()
                                ? AlignPose.Blue.Algae.poseList
                                : AlignPose.Red.Algae.poseList);
    }

    private Command run(Supplier<List<AlignPose>> poses) {
        return pointToPoint.run(
                () -> AlignPose.nearest(pose.get(), chassisSpeeds.get(), poses.get()).pose());
    }
}
