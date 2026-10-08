package frc.robot.constants;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.NetworkTableInstance;

public final class Constants {

    public static NetworkTableInstance NT_INSTANCE;

    public final class VisionConstants {


        public static String CAMERA_FR_NAME;
        public static double CAMERA_FR_YAW;
        public static double CAMERA_FR_PITCH;
        public static double CAMERA_FR_ROLL;
        public static double CAMERA_FR_FORWARD;
        public static double CAMERA_FR_LEFT;
        public static double CAMERA_FR_UP;

        public static String CAMERA_FL_NAME;
        public static double CAMERA_FL_YAW;
        public static double CAMERA_FL_PITCH;
        public static double CAMERA_FL_ROLL;
        public static double CAMERA_FL_FORWARD;
        public static double CAMERA_FL_LEFT;
        public static double CAMERA_FL_UP;

        public static String CAMERA_BR_NAME; 
        public static double CAMERA_BR_YAW;
        public static double CAMERA_BR_PITCH;
        public static double CAMERA_BR_ROLL;
        public static double CAMERA_BR_FORWARD;
        public static double CAMERA_BR_LEFT;
        public static double CAMERA_BR_UP;


        
        // Trust Constants

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static double MAX_VALID_DIST;
        public static Matrix<N3, N1> kSINGLE_TAG_STD_DEVS;
        public static Matrix<N3, N1> kMULTI_TAG_STD_DEVS;
        public static Matrix<N3, N1> ODOM_STD_DEV;


    }

    public final class SwerveConstants {
        public static Translation2d k_frontLeftLocation;
        public static Translation2d k_frontRightLocation;
        public static Translation2d k_backLeftLocation;
        public static Translation2d k_backRightLocation;
    }

    

    public final class LauncherConstants {
        public static double SHOOTER_SIZE_IN = 7.0;
    
    /** Center of shooter is 11.5 inches from the side edge */
        public static double SHOOTER_SIDE_OFFSET_IN = 11.5;

        public static double HUB_CENTER_HEIGHT = 72;

        public static double HUB_HOOD_ANGLE = 0.0;
        public static double HUB_RPM = -1725.0;

        public static double TOWER_HOOD_ANGLE = 1.25;
        public static double TOWER_RPM = -2100.0;

        public static double TEMP_AUTO_START_HOOD_ANGLE = 1.25;
        public static double TEMP_AUTO_RPM = -2300.0;

        public static double TRENCH_AUTO_START_HOOD_ANGLE = 1.25;
        public static double TRENCH_AUTO_RPM = -2200.0;

        public static double ABSOLUTE_ENCODER_OFFSET = -0.865;
        public static double ENCODER_CONVERSION = 3.942;


        public static double ROTOR_TO_SENSOR_RATIO = 4.0;
    }
}