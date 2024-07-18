// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;
/** Add your docs here. */
public class VisionOdm {

    public VisionOdm(){
// not sure if anything needs to be in here.... seems to work though...

    }
    public void updateodm(){ 
    
    
     boolean farmonedoRejectUpdate = false;
     boolean farmtwodoRejectUpdate = false;
     boolean equalNull;
     PoseEstimate bestPose;
     PoseEstimate farmone;
     PoseEstimate farmtwo;
     CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
     boolean meta2 = false;
     double AngularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
     SmartDashboard.putNumber("megatagDegrees",AngularVelocity);
     double rotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    if (meta2 == false){
     try {
       farmone = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-farmone");
       } catch (Exception e) {
        farmone = null;
       } 
     try {
       farmtwo = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-farmtwo");
       } catch (Exception e) {
        farmtwo = null;
       }
     //Limelight farmone process to reject bad pose estimites and keep the good ones 
     if (farmone !=null){
      SmartDashboard.putString("Limelight1 Status", "ready");
     if(farmone.tagCount == 1 && farmone.rawFiducials.length == 1)
      {
        if(farmone.rawFiducials[0].ambiguity > .7)
        {
          farmonedoRejectUpdate = true;
        }
        if(farmone.rawFiducials[0].distToCamera > 3)
        {
          farmonedoRejectUpdate = true;
        }
      }
      if(farmone.tagCount == 0)
      {
        farmonedoRejectUpdate = true;
      }}else{
        farmonedoRejectUpdate=true;
        SmartDashboard.putString("Limelight1 Status", "Disconnected");
    }
     //Limelight farmtwo process to reject bad pose estimites and keep the good ones
     if (farmtwo !=null){ 
      SmartDashboard.putString("Limelight2 Status", "ready");
     if(farmtwo.tagCount == 1 && farmtwo.rawFiducials.length == 1)
      {
        if(farmtwo.rawFiducials[0].ambiguity > .7)
        {
          farmtwodoRejectUpdate = true;
        }
        if(farmtwo.rawFiducials[0].distToCamera > 3)
        {
          farmtwodoRejectUpdate = true;
        }
      }
      if(farmtwo.tagCount == 0)
      {
        farmtwodoRejectUpdate = true;
      }}else{
      farmtwodoRejectUpdate = true;
      SmartDashboard.putString("Limelight2 Status", "Disconnected");
      }
      //This is where we decide what pose we want to use to update our odomatery so we check if both are vaild if so we check which has a higher tag to image area 
      if (farmonedoRejectUpdate != true && farmtwodoRejectUpdate != true){
        bestPose = (farmone.avgTagArea >= farmtwo.avgTagArea ) ? farmone : farmtwo;
      }else if (farmonedoRejectUpdate != true){
        bestPose = farmone;
      }else if(farmtwodoRejectUpdate != true) {
        bestPose = farmtwo;
      }else{bestPose=null;}
        
      if (bestPose != null) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        drivetrain.addVisionMeasurement(
            bestPose.pose,
            bestPose.timestampSeconds);}
      } else{ 
       try {LimelightHelpers.SetRobotOrientation("limelight-farmone", rotation, 0, 0, 0, 0, 0);
       farmone = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-farmone");
        
       } catch (Exception e) {
        farmone = null;
       } 
      
      if(farmone !=null){
        SmartDashboard.putString("Limelight1 Status", "ready");
       if(Math.abs(AngularVelocity) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        farmonedoRejectUpdate = true;
      }
      if(farmone.tagCount == 0)
      {
        farmonedoRejectUpdate = true;
      }
      if(!farmonedoRejectUpdate)
      {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(
            farmone.pose,
            farmone.timestampSeconds);
      }}else{
        SmartDashboard.putString("Limelight1 Status", "Disconnected");
      }
}}}
