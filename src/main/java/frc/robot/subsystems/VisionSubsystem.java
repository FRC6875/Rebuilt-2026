// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

//import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartDashboard.Field2d;
//import edu.wpi.first2.command.SubsystemBase;



//import frc.robot.utils.HardwareMonitor;
//import frc.robot.utils.Helpers;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.cameraserver.CameraServer; // kathryn messing around
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new Vision. */

  public static class VisionConstants {
    public static final String kAprilTagCam = "AprilTag_Camera";

    //Scoring offset constants(meters, robot-centric)
    //xOffset:Distance form robot center to bumper edge + camera offset

    public static final double xOffset = 0;// adjust when attached
    public static final double yOffsetRight = 0;
    public static final double yOffsetLeft = 0;

    public static final double rightAngleOffset = 0;
    public static final double leftAngleOffset = 0;

    public static final double [][] tageSpecificOffset = {{0, 0, 0, 0}} // Tag ID 0 -> figure out whats happening here

    public static final double angTolerance = 25.0/180*Math.PI; //what is this?? we dont know!

    public static final Transform3d robotToAprilTagCam = 
      new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(30)));
      
  public static final double kMaxGyroCameraAngleDelta = 89.0; //why? we dont know!!
   
  public class VisionTargetTag{
    public int tagID;
    public Pose2d pose;
  }

  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera aprilTagCam;

  //public final PhotonPoseEstimator aprilTagCam;



    

    //PhotonCamera camera = new PhotonCamera("AprilTag_Camera");
    var result = aprilTagCam.getLatestResult(); //gets most recent data from photon vision
    boolean hasTargets= result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetId = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
  
    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    //double skew = target.getSkew();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    //List<TargetCorner> corners = target.getDetectedCorners();

    



    //calculate robot's field relative pose
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()){
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    }
   


  
  
  }


/* 

  public static class VisionConstants{
    public static final String aprilTagCam = "AprilTag_Camera";

    //Scoring offset constants(meters, robot-centric)
    //xOffset:Distance form robot center to bumper edge + camera offset

    public static final double xOffset = 0;// adjust when attached
  }

  

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
