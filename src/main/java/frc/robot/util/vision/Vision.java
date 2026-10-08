package frc.robot.util.vision;

import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import frc.robot.constants.Constants;
import frc.robot.util.vision.Vision.BW.BWCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public final class Vision extends SubsystemBase{

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    private List<PhotonPipelineResult> latestResults;
    
    private Matrix <N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    public static PhotonPipelineResult latestResultCameraFR = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraFL = new PhotonPipelineResult();
    public static PhotonPipelineResult latestResultCameraBR = new PhotonPipelineResult();

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Map<BWCamera, PhotonPoseEstimator> estimators = new EnumMap<>(BWCamera.class);   
    
    private final NetworkTable visionTable;

    private final Field2d debugField = new Field2d();

    //TODO: PUBLISH VALUES TO NETWORK TABLES
    public Vision(EstimateConsumer estC) {

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        for(BWCamera cam : BW.BWCamera.values()) {
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(kTagLayout, cam.robotToCameraOffset);

            estimators.put(cam, estimator);
        }

        estConsumer = estC;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.VisionConstants.kSINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
    
    //Returns an Array of 3 EstimatedRobotPose Objects (1 per camera)
    public List<EstimatedRobotPose> getEstimatedGlobalPoses(Pose2d referencePose) {
        List<EstimatedRobotPose> allEstimates = new ArrayList<>();
        for(BWCamera cam : BWCamera.values()) {
            PhotonPoseEstimator estimator = estimators.get(cam);

            if(estimator == null) continue;

            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var result : cam.getCamera().getAllUnreadResults()) {
                visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

                visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
            }
            visionEst.ifPresent(allEstimates::add);
        }
        return allEstimates;
    }






    
    
    public boolean hasTargets() {
        if (latestResults == null || latestResults.isEmpty()) {
            return false;
        }
        for (PhotonPipelineResult r : latestResults) {
            if (r.hasTargets()) return true;
        }
        return false;
    }
    
    public List<PhotonPipelineResult> getCameraToTarget() {
        return latestResults;
    }
    
    public void processTargets(PhotonPipelineResult result){
        getCameraToTarget();{
            
        }  
     {

     List<PhotonTrackedTarget> targets = result.getTargets();{
            for (PhotonTrackedTarget target : targets) {
                double yaw = target.getYaw();
                double pitch = target.getPitch();
                double area = target.getArea();
                double skew = target.getSkew();
                //Transform2d pose = target.getCameraToTarget();
                //List<TargetCorner> corners = target.getCorners();
                // get info from target
                int targetId = target.getFiducialId();
                double poseAmbiguity = target.getPoseAmbiguity();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

                // Add additional processing as needed
            }       
        } 
    }
    }

    //CAMERA ENUM
    public static final class BW {
        public enum BWCamera {
            CAMERA_FR (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_FR_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_FR_FORWARD,
                    Constants.VisionConstants.CAMERA_FR_LEFT,
                    Constants.VisionConstants.CAMERA_FR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FR_YAW)
                    )
                )
            ),
            
            CAMERA_FL (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_FL_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_FL_FORWARD,
                    Constants.VisionConstants.CAMERA_FL_LEFT,
                    Constants.VisionConstants.CAMERA_FL_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_FL_YAW)
                    )
                )
            ),

            CAMERA_BR (
                new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.CAMERA_BR_NAME),
                new Transform3d(
                    Constants.VisionConstants.CAMERA_BR_FORWARD,
                    Constants.VisionConstants.CAMERA_BR_LEFT,
                    Constants.VisionConstants.CAMERA_BR_UP,
                    new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.CAMERA_BR_YAW)
                    )
                )
            );


            final PhotonCamera camera; 
            final Transform3d robotToCameraOffset;
            BWCamera(PhotonCamera camera, Transform3d robotToCameraOffset) {
                this.camera = camera; 
                this.robotToCameraOffset = robotToCameraOffset;
            }

            public PhotonCamera getCamera(){
                return camera;
            }
            
            public Transform3d getRobotToCameraOffset(){
                return robotToCameraOffset;
            }
        }


    } 



    public static PhotonPipelineResult getLatestResult(BWCamera camera){
        return switch (camera){
            case CAMERA_FR -> latestResultCameraFR;
            case CAMERA_FL -> latestResultCameraFL;
            case CAMERA_BR -> latestResultCameraBR;
        };
    }
    public static boolean hasTargets(BWCamera camera) {
        return switch (camera) {
            case CAMERA_FR -> latestResultCameraFR.hasTargets();
            case CAMERA_FL -> latestResultCameraFL.hasTargets();
            case CAMERA_BR -> latestResultCameraBR.hasTargets();
        };
        
    }
}