// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Math Imports
import edu.wpi.first.math.util.Units; //Converts motor position to m/s
public final class Constants {
  //Driver controller port
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //SparkMax Can Id's
  public static class CanIDConstants{
    public static final int kFrontLeftChannel = 1; //ID 1
    public static final int kRearLeftChannel = 3; //ID 3
    public static final int kFrontRightChannel = 2; //ID 2
    public static final int kRearRightChannel = 4; //ID 4
  }

  //Conversion formula from motor speed to m/s  
  public static class DriveConstants {
    public static final double kLinearDistanceConverter = 2 * Math.PI * Units.inchesToMeters(3) / 27;
  }

}