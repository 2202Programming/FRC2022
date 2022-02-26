// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/** Common Pose-based calculations */
public class PoseMath {

  /*
   * poseDistance(x) - returns distance between two poses
   *              
   * @param pose1
   * @param pose2  
   * @return double
  */
  public static double poseDistance(Pose2d pose1, Pose2d pose2){
          return Math.sqrt(
          (Math.pow(pose1.getX() - pose2.getX(),2)) +
          (Math.pow(pose1.getY() - pose2.getY(),2)));
  }

}
