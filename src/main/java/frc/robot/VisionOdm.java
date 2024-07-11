// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;

/** Add your docs here. */
public class VisionOdm {

    public VisionOdm(){
// not sure if anything needs to be in here.... seems to work though...

    }
    public void updateodm(){ 
    
    
      boolean doRejectUpdate = false;
     boolean equalNull;
     CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
     
    
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
      }
}}
