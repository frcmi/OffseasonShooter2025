package team5937.frc2025.autoalign;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import team5937.frc2025.constants.AutoAlignConstants;
import team5937.frc2025.constants.VisionConstants;

import java.util.*;

public record AlignPose(int id, Pose2d pose) {
    private static final double coralTranslation = Units.inchesToMeters(18.25);
    private static final double algaeTranslation = Units.inchesToMeters(17.5);
    private static final Transform2d toLeftSide =
            new Transform2d(
                    new Translation2d(coralTranslation, -AutoAlignConstants.kAprilTagToReefDist),
                    Rotation2d.k180deg);
    private static final Transform2d toRightSide =
            new Transform2d(
                    new Translation2d(coralTranslation, AutoAlignConstants.kAprilTagToReefDist),
                    Rotation2d.k180deg);
    private static final Transform2d toCenter =
            new Transform2d(new Translation2d(algaeTranslation, 0), Rotation2d.k180deg);

    private static int globalId = 0;

    public AlignPose(Pose2d pose) {
        this(globalId++, pose);
    }

    public static AlignPose nearest(Pose2d pose, ChassisSpeeds speeds, List<AlignPose> poses) {
        return Collections.min(
                poses,
                Comparator.comparing(
                                (AlignPose other) ->
                                        pose.getTranslation()
                                                .getDistance(other.pose().getTranslation()))
                        .thenComparing(
                                (AlignPose other) ->
                                        Math.abs(
                                                pose.getRotation()
                                                        .minus(other.pose().getRotation())
                                                        .getRadians())));
    }

    public static class Blue {
        public static class Coral {
            public static final AlignPose A =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(18)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose B =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(18)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose C =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(17)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose D =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(17)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose E =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(22)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose F =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(22)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose G =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(21)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose H =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(21)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose I =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(20)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose J =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(20)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose K =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(19)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose L =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(19)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final Map<Integer, AlignPose> posesMap = new HashMap<>(12);
            public static List<AlignPose> posesList = new ArrayList<>();

            static {
                posesMap.put(A.id, A);
                posesMap.put(B.id, B);
                posesMap.put(C.id, C);
                posesMap.put(D.id, D);
                posesMap.put(E.id, E);
                posesMap.put(F.id, F);
                posesMap.put(G.id, G);
                posesMap.put(H.id, H);
                posesMap.put(I.id, I);
                posesMap.put(J.id, J);
                posesMap.put(K.id, K);
                posesMap.put(L.id, L);
                posesList.addAll(posesMap.values());
            }
        }

        public static class Algae {
            public static final AlignPose AB =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(18)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose CD =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(17)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose EF =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(22)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose GH =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(21)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose IJ =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(20)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose KL =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(19)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final List<AlignPose> poseList = List.of(AB, CD, EF, GH, IJ, KL);
        }
    }

    public static class Red {
        public static class Coral {
            public static final AlignPose A =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(7)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose B =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(7)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose C =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(8)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose D =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(8)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose E =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(9)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose F =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(9)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose G =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(10)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose H =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(10)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose I =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(11)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose J =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(11)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final AlignPose K =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(6)
                                    .toPose2d()
                                    .transformBy(toLeftSide));
            public static final AlignPose L =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(6)
                                    .toPose2d()
                                    .transformBy(toRightSide));
            public static final Map<Integer, AlignPose> posesMap = new HashMap<>(12);
            public static final List<AlignPose> posesList = new ArrayList<>();

            static {
                posesMap.put(A.id, A);
                posesMap.put(B.id, B);
                posesMap.put(C.id, C);
                posesMap.put(D.id, D);
                posesMap.put(E.id, E);
                posesMap.put(F.id, F);
                posesMap.put(G.id, G);
                posesMap.put(H.id, H);
                posesMap.put(I.id, I);
                posesMap.put(J.id, J);
                posesMap.put(K.id, K);
                posesMap.put(L.id, L);
                posesList.addAll(posesMap.values());
            }
        }

        public static class Algae {
            public static final AlignPose AB =
                    new AlignPose(
                            VisionConstants.kAprilTagField.get(7).toPose2d().transformBy(toCenter));
            public static final AlignPose CD =
                    new AlignPose(
                            VisionConstants.kAprilTagField.get(8).toPose2d().transformBy(toCenter));
            public static final AlignPose EF =
                    new AlignPose(
                            VisionConstants.kAprilTagField.get(9).toPose2d().transformBy(toCenter));
            public static final AlignPose GH =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(10)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose IJ =
                    new AlignPose(
                            VisionConstants.kAprilTagField
                                    .get(11)
                                    .toPose2d()
                                    .transformBy(toCenter));
            public static final AlignPose KL =
                    new AlignPose(
                            VisionConstants.kAprilTagField.get(6).toPose2d().transformBy(toCenter));
            public static final List<AlignPose> poseList = List.of(AB, CD, EF, GH, IJ, KL);
        }
    }
}
