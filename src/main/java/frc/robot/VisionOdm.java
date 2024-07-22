// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;
/** Add your docs here. */
public class VisionOdm {

  boolean farmOneDoRejectUpdate = false;
     boolean farmTwoDoRejectUpdate = false;
     boolean equalNull;
     //Pose Estimation determines a robot’s position and orientation on the field by fusing sensor data, accounting for drift and noise, and providing accurate, latency-compensated estimates for different drivetrains.
     PoseEstimate bestPose;
     PoseEstimate farmOne;
     PoseEstimate farmTwo;
     CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
     // Meta MegaTag2 offers more precise and reliable localization compared to the original MegaTag system
     boolean meta2 = false; //false means you will use the orginal meta while true means you will use MegaTag2 
     
     double AngularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
     
     double rotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    public VisionOdm(){
      
// not sure if anything needs to be in here.... seems to work though...

    }
    public void updateodm(){ 
    if (meta2 == false){
     try {
      // We are assigning the limelight named "farmone" to the PoseEstimate "farmOne"
       farmOne = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-farmone");
       } catch (Exception e) {
        farmOne = null;
       } 
     try {
      // We are assigning the limelight named "farmtwo" to the PoseEstimate "farmTwo"
       farmTwo = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-farmtwo");
       } catch (Exception e) {
        farmTwo = null;
       }
     //Limelight farmone process to reject bad pose estimites and keep the good ones 
     if (farmOne !=null){
      SmartDashboard.putString("Limelight1 Status", "ready");
     if(farmOne.tagCount == 1 && farmOne.rawFiducials.length == 1)
      {
        if(farmOne.rawFiducials[0].ambiguity > .7)
        {
          farmOneDoRejectUpdate = true;
        }
        if(farmOne.rawFiducials[0].distToCamera > 3)
        {
          farmOneDoRejectUpdate = true;
        }
      }
      if(farmOne.tagCount == 0)
      {
        farmOneDoRejectUpdate = true;
      }}else{
        farmOneDoRejectUpdate=true;
        SmartDashboard.putString("Limelight1 Status", "Disconnected");
    }
     //Limelight farmtwo process to reject bad pose estimites and keep the good ones
     if (farmTwo !=null){ 
      SmartDashboard.putString("Limelight2 Status", "ready");
     if(farmTwo.tagCount == 1 && farmTwo.rawFiducials.length == 1)
      {
        if(farmTwo.rawFiducials[0].ambiguity > .7)
        {
          farmTwoDoRejectUpdate = true;
        }
        if(farmTwo.rawFiducials[0].distToCamera > 3)
        {
          farmTwoDoRejectUpdate = true;
        }
      }
      if(farmTwo.tagCount == 0)
      {
        farmTwoDoRejectUpdate = true;
      }}else{
      farmTwoDoRejectUpdate = true;
      SmartDashboard.putString("Limelight2 Status", "Disconnected");
      }
      // This is the procces to take two limelights posees and find the Pose thats the closest to a april tag and use that pose. If one is null then the other is the presumed best pose. If both are null then best pose = NULL aswell.
      if (farmOneDoRejectUpdate != true && farmTwoDoRejectUpdate != true){
        bestPose = (farmOne.avgTagArea >= farmTwo.avgTagArea ) ? farmOne : farmTwo;
      }else if (farmOneDoRejectUpdate != true){
        bestPose = farmOne;
      }else if(farmTwoDoRejectUpdate != true) {
        bestPose = farmTwo;
      }else{bestPose=null;}
        
      if (bestPose != null) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        drivetrain.addVisionMeasurement(
            bestPose.pose,
            bestPose.timestampSeconds);}
      } else{ 
        // if meta2 = true this will run 
        // We are assigning the limelight named "farmone" to the PoseEstimate "farmOne"
       try {LimelightHelpers.SetRobotOrientation("limelight-farmone", rotation, 0, 0, 0, 0, 0);
       farmOne = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-farmone");
        
       } catch (Exception e) {
        farmOne = null;
       } 
      
        if(farmOne !=null){
          SmartDashboard.putString("Limelight1 Status", "ready");
        if(Math.abs(AngularVelocity) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          farmOneDoRejectUpdate = true;
        }
        if(farmOne.tagCount == 0)
        {
          farmOneDoRejectUpdate = true;
        }
        if(!farmOneDoRejectUpdate)
        {
          drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          drivetrain.addVisionMeasurement(
              farmOne.pose,
              farmOne.timestampSeconds);
      }}else{
        SmartDashboard.putString("Limelight1 Status", "Disconnected");
      }
}}}
