package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import static edu.wpi.first.units.Units.Degrees;

public class Constants {
    public static final boolean REPLAY = false;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static class VisionConstants {
        // THE translations FOR THE LEFT AND RIGHT CAMERAS
        public static Translation3d l_camera_translation = new Translation3d(0.055, -0.035, 0.263);
        public static Translation3d r_camera_translation = new Translation3d(0.05, 0.067, 0.263);

        // THE ROTATION FOR THE LEFT AND RIGHT CAMERAS
        public static Rotation3d l_camera_rotation = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-16.7));
        public static Rotation3d r_camera_rotation = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(14.5));

        // THE TRANSFORMATION FOR THE LEFT AND RIGHT CAMERAS
        public static Transform3d l_camera_transform = new Transform3d(l_camera_translation, l_camera_rotation);
        public static Transform3d r_camera_transform = new Transform3d(r_camera_translation, r_camera_rotation);

        public static final Transform2d transformRightBranch = new Transform2d(0.384, 0.013, new Rotation2d(-0.1));
        public static final Transform2d transformLeftBranch = new Transform2d(0.384, 0.013, new Rotation2d(-0.1));
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
