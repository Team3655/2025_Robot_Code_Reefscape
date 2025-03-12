package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants.ObservationType;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

public class VisionIOSim implements VisionIO {

  private final VisionSystemSim visionSystem;
  private final SimCameraProperties cameraProperties;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator poseEstimator;

  public final String name;
  private AprilTagFieldLayout tagLayout;

  /**
   * Uses PhotonLib to simulate vision calculations.
   * 
   * @param name          The name of the camera
   * @param robotToCamera A Transform3d representing the relative position of the
   *                      camera compared to the robot
   */
  public VisionIOSim(String name, Transform3d robotToCamera) {

    this.name = name;
    // Initialize April Tag layout
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load april tags :3 !!", null);
    }

    // Simulates a camera and a coprocessor running PhotonLib
    visionSystem = new VisionSystemSim("system" + name);

    cameraProperties = new SimCameraProperties();

    cameraProperties.setCalibError(0.25, 0.08);
    cameraProperties.setRandomSeed((long) 0.011);

    // Create a real camera to get simulated data from
    camera = new PhotonCamera(name);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    // Enable the raw and processed streams for the cameras
    // Access processed stream with localhost:1182
    // Access raw stream with localhost:1181
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);

    // Enable drawing a wireframe visualization of the field to the camera
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(false);

    // Add the april tags as vision targets
    visionSystem.addAprilTags(tagLayout);

    // Add the camera to the system
    visionSystem.addCamera(cameraSim, robotToCamera);

    // Create PoseEstimator
    poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    inputs.name = name;

    inputs.connected = camera.isConnected();

    // Update the systems position
    visionSystem.update(RobotState.getInstance().getOdometryPose());

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Get all unread vision measurements.
    for (var result : camera.getAllUnreadResults()) {

      if (result.hasTargets()) {
        // Update latest observation tx and ty, can later be used for other mechanisms
        // if desired.
        inputs.latestObservation = new TargetObservation(
            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        continue;
      }

      // More than one april tag
      if (result.multitagResult.isPresent()) {

        var multiTagResult = result.multitagResult.get();

        // Calculate total distance to tags
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag ids to a list
        tagIds.addAll(multiTagResult.fiducialIDsUsed);

        // Get pose estimation
        var estimation = getEstimatedGlobalPose(RobotState.getInstance().getEstimatedPose(), result);

        if (estimation.isPresent()) {

          // Add pose estimation to observations
          poseObservations.add(new PoseObservation(
              estimation.get().timestampSeconds,
              estimation.get().estimatedPose.toPose2d(),
              multiTagResult.estimatedPose.ambiguity,
              multiTagResult.fiducialIDsUsed.size(),
              totalTagDistance / result.targets.size(), ObservationType.PHOTON));
        }

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);
        var tagPose = tagLayout.getTagPose(target.fiducialId);

        if (tagPose.isPresent()) {
          Transform3d cameraToTarget = target.bestCameraToTarget;
          tagIds.add((short) target.fiducialId);

          // Get pose estimation
          var estimation = getEstimatedGlobalPose(RobotState.getInstance().getEstimatedPose(), result);
          // Use PhotonLib's built in PoseEstimator.
          if (estimation.isPresent()) {
            poseObservations.add(new PoseObservation(
                estimation.get().timestampSeconds,
                estimation.get().estimatedPose.toPose2d(),
                target.poseAmbiguity,
                1,
                cameraToTarget.getTranslation().getNorm(), ObservationType.PHOTON));
          }
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Gets an estimated pose from PhotonLib's pose estimator
   * 
   * @param previousPose The pose where the robot used to be.
   * @param result       The result to calculate pose with
   * @return An optional representing the estimated pose
   */
  private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousPose, PhotonPipelineResult result) {
    poseEstimator.setReferencePose(previousPose);
    return poseEstimator.update(result);
  }
}
