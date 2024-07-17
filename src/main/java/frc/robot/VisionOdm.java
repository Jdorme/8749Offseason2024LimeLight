// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** Add your docs here. */
public class VisionOdm {

    public VisionOdm(){
// not sure if anything needs to be in here.... seems to work though...

    }
    public void updateodm(){ 
    
    
     boolean doRejectUpdate = false;
     boolean equalNull;
     CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
     boolean meta2 = true;
     double AngularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
     SmartDashboard.putNumber("megatagDegrees",AngularVelocity);
     double rotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    if (meta2 == false){
     LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-farmone");
       if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }} else{ 
      LimelightHelpers.SetRobotOrientation("limelight-farmone", rotation, 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-farmone");
      if(Math.abs(AngularVelocity) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }}
}}
