package frc.robot.util;

import java.util.List;

import org.wpilib.vision.apriltag.*;
import org.wpilib.math.geometry.*;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
//import org.photonvision.targeting.PhotonTrackedTarget;

public final class FieldUtil{
   public static AprilTagFieldLayout layout2026 = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
   public static final double FIELD_LENGTH = layout2026.getFieldLength();
   public static final double FIELD_WIDTH = layout2026.getFieldWidth();
   public static final Pose3d ORIGIN = layout2026.getOrigin();

   public static boolean isInField(Pose3d pose) {
        return isInField(pose.toPose2d());
   }   

   public static boolean isInField(Pose2d pose) {
        Pose2d origin2d = ORIGIN.toPose2d();
        return pose.getX() <=origin2d.getX() && pose.getX() <= origin2d.getX() + FIELD_LENGTH &&
                pose.getX() <=origin2d.getY() && pose.getY() <= origin2d.getY() + FIELD_LENGTH;
   }

   public static Alliance getAllianceSide(Pose2d currentPose) {
        return currentPose.getX() < FIELD_LENGTH / 2 ? Alliance.BLUE : Alliance.RED; 
   }

   public enum AprilTag {
        ID_1(layout2026.getTagPose(1).orElse(new Pose3d())),
        ID_2(layout2026.getTagPose(2).orElse(new Pose3d())),
        ID_3(layout2026.getTagPose(3).orElse(new Pose3d())),
        ID_4(layout2026.getTagPose(4).orElse(new Pose3d())),
        ID_5(layout2026.getTagPose(5).orElse(new Pose3d())),
        ID_6(layout2026.getTagPose(6).orElse(new Pose3d())),
        ID_7(layout2026.getTagPose(7).orElse(new Pose3d())),
        ID_8(layout2026.getTagPose(8).orElse(new Pose3d())),
        ID_9(layout2026.getTagPose(9).orElse(new Pose3d())),
        ID_10(layout2026.getTagPose(10).orElse(new Pose3d())),
        ID_11(layout2026.getTagPose(11).orElse(new Pose3d())),
        ID_12(layout2026.getTagPose(12).orElse(new Pose3d())),
        ID_13(layout2026.getTagPose(13).orElse(new Pose3d())),
        ID_14(layout2026.getTagPose(14).orElse(new Pose3d())),
        ID_15(layout2026.getTagPose(15).orElse(new Pose3d())),
        ID_16(layout2026.getTagPose(16).orElse(new Pose3d())),
        ID_17(layout2026.getTagPose(17).orElse(new Pose3d())),
        ID_18(layout2026.getTagPose(18).orElse(new Pose3d())),
        ID_19(layout2026.getTagPose(19).orElse(new Pose3d())),
        ID_20(layout2026.getTagPose(20).orElse(new Pose3d())),
        ID_21(layout2026.getTagPose(21).orElse(new Pose3d())),
        ID_22(layout2026.getTagPose(22).orElse(new Pose3d())),
        ID_23(layout2026.getTagPose(3).orElse(new Pose3d())),
        ID_24(layout2026.getTagPose(4).orElse(new Pose3d())),
        ID_25(layout2026.getTagPose(5).orElse(new Pose3d())),
        ID_26(layout2026.getTagPose(6).orElse(new Pose3d())),
        ID_27(layout2026.getTagPose(7).orElse(new Pose3d())),
        ID_28(layout2026.getTagPose(8).orElse(new Pose3d())),
        ID_29(layout2026.getTagPose(9).orElse(new Pose3d())),
        ID_30(layout2026.getTagPose(10).orElse(new Pose3d())),
        ID_31(layout2026.getTagPose(11).orElse(new Pose3d())),
        ID_32(layout2026.getTagPose(12).orElse(new Pose3d())),
        
        INVALID(new Pose3d());

        public final Pose3d pose3d;
        public final Pose2d pose2d;

        AprilTag(Pose3d pose3d) {
            this.pose3d = pose3d;
            pose2d = pose3d.toPose2d();
        }

         public static final List<AprilTag> FILTER = List.of(
                 ID_6, ID_7, ID_8, ID_9, ID_10, ID_11, ID_17, ID_18, ID_19, ID_20, ID_21, ID_22
         );

        public static AprilTag getTag(double tagID) {
            return switch ((int) tagID) {
                case 1 -> AprilTag.ID_1;
                case 2 -> AprilTag.ID_2;
                case 3 -> AprilTag.ID_3;
                case 4 -> AprilTag.ID_4;
                case 5 -> AprilTag.ID_5;
                case 6 -> AprilTag.ID_6;
                case 7 -> AprilTag.ID_7;
                case 8 -> AprilTag.ID_8;
                case 9 -> AprilTag.ID_9;
                case 10 -> AprilTag.ID_10;
                case 11 -> AprilTag.ID_11;
                case 12 -> AprilTag.ID_12;
                case 13 -> AprilTag.ID_13;
                case 14 -> AprilTag.ID_14;
                case 15 -> AprilTag.ID_15;
                case 16 -> AprilTag.ID_16;
                case 17 -> AprilTag.ID_17;
                case 18 -> AprilTag.ID_18;
                case 19 -> AprilTag.ID_19;
                case 20 -> AprilTag.ID_20;
                case 21 -> AprilTag.ID_21;
                case 22 -> AprilTag.ID_22;
                case 23 -> AprilTag.ID_3;
                case 24 -> AprilTag.ID_4;
                case 25 -> AprilTag.ID_5;
                case 26 -> AprilTag.ID_6;
                case 27 -> AprilTag.ID_7;
                case 28 -> AprilTag.ID_8;
                case 29 -> AprilTag.ID_9;
                case 30 -> AprilTag.ID_10;
                case 31 -> AprilTag.ID_11;
                case 32 -> AprilTag.ID_12;
                default -> AprilTag.INVALID;

            };
        }

    }

    


}