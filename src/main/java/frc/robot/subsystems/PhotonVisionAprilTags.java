// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionAprilTags extends SubsystemBase {
  /** Creates a new PhotonVisionAprillTags. */
  PhotonCamera camera = null;
  public PhotonVisionAprilTags() {
    camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  }

  public void processTags() {
    var result = camera.getLatestResult();
    Logger.recordOutput("AllSensors/AprilTags/CameraConnected", camera.isConnected());
    boolean hasTargets = result.hasTargets();
    Logger.recordOutput("AllSensors/AprilTags/CameraHasTarget", hasTargets);
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      Logger.recordOutput("AllSensors/AprilTags/TargetYaw", target.getYaw());
      Logger.recordOutput("AllSensors/AprilTags/TargetPitch", target.getPitch());
      Logger.recordOutput("AllSensors/AprilTags/TargetArea", target.getArea());
      Logger.recordOutput("AllSensors/AprilTags/TargetSkew", target.getSkew());
      Transform3d pose = target.getBestCameraToTarget();
      Logger.recordOutput("AllSensors/AprilTags/TargetPoseX", pose.getX());
      Logger.recordOutput("AllSensors/AprilTags/TargetPoseY", pose.getY());
      Logger.recordOutput("AllSensors/AprilTags/TargetPoseZ", pose.getZ());
      Logger.recordOutput("AllSensors/AprilTags/TargetMeasureX", pose.getMeasureX());
      Logger.recordOutput("AllSensors/AprilTags/TargetMeasureY", pose.getMeasureY());
      Logger.recordOutput("AllSensors/AprilTags/TargetMeasureZ", pose.getMeasureZ());

      double targetRange = getDistanceToAprilTag(target);
      Logger.recordOutput("AllSensors/AprilTags/TargetDistance", targetRange);

    } else { 
      
    }
  }  

  public double getDistanceToAprilTag(PhotonTrackedTarget currentTarget) {
    double cameraHeight = 0.387;
    double targetHeight = 0.21;
    double cameraPitch  = -8.9;

    double targetPitch = currentTarget.getPitch();
    double targetRange = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, Units.degreesToRadians(cameraPitch), Units.degreesToRadians(targetPitch));
  return targetRange;
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    processTags();
  }
}
