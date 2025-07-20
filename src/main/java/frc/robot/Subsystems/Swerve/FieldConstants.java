package frc.robot.Subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of
                                                                              // starting
                                                                              // line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static class Processor {
        public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0,
                Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(286.779));
        public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(242.855));
        public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428),
                Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace = new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(291.176),
                Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace = new Pose2d(
                Units.inchesToMeters(33.526),
                Units.inchesToMeters(25.824),
                Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
                Units.inchesToMeters(158.501));
        public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside
                                                                              // of the
                                                                              // reef zone line

        public static final Pose2d[] blueCenterFaces = new Pose2d[6]; // Starting facing the driver station in
                                                                      // clockwise
                                                                      // order
        public static final Pose2d[] redCenterFaces = new Pose2d[6]; // Starting facing the driver station in
                                                                     // clockwise
                                                                     // order
        public static final Pose2d[] blueLeftBranches = new Pose2d[6]; // Starting facing the driver station in
                                                                       // clockwise
                                                                       // order
        public static final Pose2d[] blueRightBranches = new Pose2d[6]; // Starting facing the driver station in
                                                                        // clockwise
                                                                        // order
        public static final Pose2d[] redLeftBranches = new Pose2d[6]; // Starting facing the driver station in
                                                                      // clockwise
                                                                      // order
        public static final Pose2d[] redRightBranches = new Pose2d[6]; // Starting facing the driver station in
                                                                       // clockwise
                                                                       // order
        public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(); // Starting at
                                                                                               // the right
                                                                                               // branch facing
                                                                                               // the
                                                                                               // driver station
                                                                                               // in
                                                                                               // clockwise
        public static final double branchDist = 0.16; // Half the distance between branches

        static {
            // Initialize faces
            blueCenterFaces[0] = new Pose2d(
                    Units.inchesToMeters(144.003 - 16.04),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(0));
            blueCenterFaces[1] = new Pose2d(
                    Units.inchesToMeters(160.373 - (16.04 * Math.cos(Math.PI / 3))),
                    Units.inchesToMeters(186.857 + (16.04 * Math.sin(Math.PI / 3))),
                    Rotation2d.fromDegrees(60));
            blueCenterFaces[2] = new Pose2d(
                    Units.inchesToMeters(193.116 - (16.04 * Math.cos(2 * Math.PI / 3))),
                    Units.inchesToMeters(186.858 + (16.04 * Math.sin(2 * Math.PI / 3))),
                    Rotation2d.fromDegrees(120));
            blueCenterFaces[3] = new Pose2d(
                    Units.inchesToMeters(209.489 + 16.04),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(180));
            blueCenterFaces[4] = new Pose2d(
                    Units.inchesToMeters(193.118 - (16.04 * Math.cos(-2 * Math.PI / 3))),
                    Units.inchesToMeters(130.145 + (16.04 * Math.sin(-2 * Math.PI / 3))),
                    Rotation2d.fromDegrees(-120));
            blueCenterFaces[5] = new Pose2d(
                    Units.inchesToMeters(160.375 - (16.04 * Math.cos(-Math.PI / 3))),
                    Units.inchesToMeters(130.144 + (16.04 * Math.sin(-Math.PI / 3))),
                    Rotation2d.fromDegrees(-60));
            Logger.recordOutput("center face 0", blueCenterFaces);
            for (int i = 0; i < 6; i++) {

                blueRightBranches[i] = new Pose2d(
                        blueCenterFaces[i].getX() - branchDist
                                * Math.sin(blueCenterFaces[i].getRotation()
                                        .getRadians()),
                        blueCenterFaces[i].getY() - branchDist
                                * Math.cos(blueCenterFaces[i].getRotation()
                                        .getRadians()),
                        Rotation2d.fromDegrees(-i * 60));
                blueLeftBranches[i] = new Pose2d(
                        blueCenterFaces[i].getX() + branchDist
                                * Math.sin(blueCenterFaces[i].getRotation()
                                        .getRadians()),
                        blueCenterFaces[i].getY() + branchDist
                                * Math.cos(blueCenterFaces[i].getRotation()
                                        .getRadians()),
                        Rotation2d.fromDegrees(-i * 60));

            }
            Logger.recordOutput("left branch blue", blueLeftBranches);
            Logger.recordOutput("right branch blue", blueRightBranches);

            for (int i = 0; i < blueCenterFaces.length; i++) {
                redCenterFaces[i] = new Pose2d(fieldLength - blueCenterFaces[i].getX(),
                        fieldWidth - blueCenterFaces[i].getY(),
                        Rotation2d.fromDegrees(
                                180 + blueCenterFaces[i].getRotation().getDegrees()));
                redLeftBranches[i] = new Pose2d(fieldLength - blueLeftBranches[i].getX(),
                        fieldWidth - blueLeftBranches[i].getY(),
                        Rotation2d.fromDegrees(
                                180 + blueLeftBranches[i].getRotation().getDegrees()));
                redRightBranches[i] = new Pose2d(fieldLength - blueRightBranches[i].getX(),
                        fieldWidth - blueRightBranches[i].getY(),
                        Rotation2d.fromDegrees(
                                180 + blueRightBranches[i].getRotation()
                                        .getDegrees()));
            }
            Logger.recordOutput("left branch 0", redLeftBranches);
            // leftBranches[1] = new Pose2d(
            // branchDist * Math.sin(-2 * Math.PI / 3) + centerFaces[1].getX(),
            // branchDist * Math.cos(-2 * Math.PI / 3) + centerFaces[1].getY(),
            // new Rotation2d(-2 * Math.PI / 3));
            // SmartDashboard.putNumber("left1 x transform", new Transform2d(
            // branchDist * Math.sin(-2 * Math.PI / 3),
            // branchDist * Math.cos(-2 * Math.PI / 3),
            // new Rotation2d()).getX());

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection = new Pose2d(center,
                            Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            adjustY,
                                                            new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            adjustY,
                                                            new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(
                                                    level.pitch),
                                            poseDirection.getRotation()
                                                    .getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            -adjustY,
                                                            new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(
                                                            adjustX,
                                                            -adjustY,
                                                            new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(
                                                    level.pitch),
                                            poseDirection.getRotation()
                                                    .getRadians())));
                }
                branchPositions.add(fillRight);
                branchPositions.add(fillLeft);
            }
        }

        public static class StagingPositions {
            // Measured from the center of the ice cream
            public static final Pose2d leftIceCream = new Pose2d(Units.inchesToMeters(48),
                    Units.inchesToMeters(230.5),
                    new Rotation2d());
            public static final Pose2d middleIceCream = new Pose2d(Units.inchesToMeters(48),
                    Units.inchesToMeters(158.5),
                    new Rotation2d());
            public static final Pose2d rightIceCream = new Pose2d(Units.inchesToMeters(48),
                    Units.inchesToMeters(86.5),
                    new Rotation2d());
        }

        public enum ReefHeight {
            L1(Units.inchesToMeters(25.0), 0),
            L2(Units.inchesToMeters(31.875), -35),
            L3(Units.inchesToMeters(47.625), -35),
            L4(Units.inchesToMeters(72), -90);

            ReefHeight(double height, double pitch) {
                this.height = height;
                this.pitch = pitch; // in degrees
            }

            public static ReefHeight fromLevel(int level) {
                return Arrays.stream(values())
                        .filter(height -> height.ordinal() == level)
                        .findFirst()
                        .orElse(L4);
            }

            public final double height;
            public final double pitch;
        }

        public static final double aprilTagWidth = Units.inchesToMeters(6.50);
        public static final int aprilTagCount = 22;
        public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.NO_BARGE;

        public enum AprilTagLayoutType {
            OFFICIAL("2025-official"),
            NO_BARGE("2025-no-barge"),
            BLUE_REEF("2025-blue-reef"),
            RED_REEF("2025-red-reef");

            AprilTagLayoutType(String name) {

                try {
                    layout = new AprilTagFieldLayout(
                            Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags",
                                    name + ".json"));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }

                if (layout == null) {
                    layoutString = "";
                } else {
                    try {
                        layoutString = new ObjectMapper().writeValueAsString(layout);
                    } catch (JsonProcessingException e) {
                        throw new RuntimeException(
                                "Failed to serialize AprilTag layout JSON " + toString()
                                        + "for Northstar");
                    }
                }
            }

            private final AprilTagFieldLayout layout;
            private final String layoutString;
        }

        public record CoralObjective(int branchId, ReefHeight reefLevel) {
        }
    }
}
