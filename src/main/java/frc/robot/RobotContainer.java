// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants; //holds the driver controller
import frc.robot.commands.DriveMecanum; //Drives the robot with driveCartesian(), driveCartesianIK(), or drivePolar() 
import frc.robot.subsystems.DriveSubsystem; //Connects the subsystem file to the subsystem framework

//Controller imports
import edu.wpi.first.wpilibj.Joystick; //Get Joysick values
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; //Get Xbox values
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Get Joystick/Xbox buttons

//Command Imports
import edu.wpi.first.wpilibj2.command.Command; //Simple command with start and end condition

public class RobotContainer {
  // The robot's subsystems and commands are defined 
  public final static DriveSubsystem m_Drivetrain = new DriveSubsystem();

  // Initlize the controller from driverstation using the port constant
  public static Joystick driverController = new Joystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    m_Drivetrain.setDefaultCommand(new DriveMecanum());
    configureBindings();
  }

  //Define the bindings and what command/method they call. 
  private void configureBindings() {
    new JoystickButton(driverController, 1).onTrue(m_Drivetrain.toggleFO()); //Drive A, toggle field orientated drive
    new JoystickButton(driverController, 3).onTrue(m_Drivetrain.followTrajectoryCommand(m_Drivetrain.traj, true)); //Call the follow path command
  }

  //Ignore
  public Command getAutonomousCommand() {  
    return null;
  }
}
