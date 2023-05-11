// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanum. */


  public DriveMecanum() {
    addRequirements(RobotContainer.m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled. (20ms)
  @Override
  public void execute() {
    //Check if field orintated drive is active 
    boolean FieldOrintated = DriveSubsystem.getFODrive();

    //Get the XboxController inputs 
    double xSpeed = RobotContainer.driverController.getY();
    double ySpeed = -RobotContainer.driverController.getX();
    double zRotation = -RobotContainer.driverController.getZ();

    //Get Gyro Angle
    Rotation2d gyroAngle = DriveSubsystem.ahrs.getRotation2d();

    //If field orintated drive is active, overload the Drive() method
    if (FieldOrintated) {
      RobotContainer.m_Drivetrain.brakeMode(false);
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, zRotation, gyroAngle);
    }
    else {
      if (zRotation > 0.25 || zRotation < -0.25){
      RobotContainer.m_Drivetrain.brakeMode(false);
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed  , zRotation);
      }
      else{
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, 0);
      RobotContainer.m_Drivetrain.brakeMode(false);

      }
      }
    }
  
  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
