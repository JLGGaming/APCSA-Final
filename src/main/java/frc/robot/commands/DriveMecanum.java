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
    double xSpeed = -RobotContainer.driverController.getLeftY();
    double ySpeed = -RobotContainer.driverController.getLeftX();
    double zRotation = -RobotContainer.driverController.getRightX();

    //Get Gyro Angle
    Rotation2d gyroAngle = DriveSubsystem.ahrs.getRotation2d();

    //If field orintated drive is active, overload the Drive() method
    if (FieldOrintated) {
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, zRotation, gyroAngle);
    }
    else {
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, zRotation);
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
