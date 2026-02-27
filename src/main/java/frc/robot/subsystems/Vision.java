// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer; // kathryn messing around
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {

    PhotonCamera camera = new PhotonCamera("AprilTag_Camera");
    var result = camera.getLatestResult(); //gets most recent data from photon vision
    boolean hasTargets= result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
   
    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    Transform3d pose = target.getBestCameraToTarget();
    List<TargetCorner> corners = target.getDetectedCorners();

  }
  /* 
  public class OrangePiVision {
    public static void main(String... args){
      NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
      ntinst.setServerTeam(6875);
      
      CameraServer.startAutomaticCapture();

      while (!Thread.interrupted());
    }
  }
    */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
