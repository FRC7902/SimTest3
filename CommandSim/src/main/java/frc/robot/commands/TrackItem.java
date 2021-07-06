// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TrackItem extends CommandBase {
  private final CameraSubsystem m_cameraSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  /** Creates a new TrackItem. */
  public TrackItem(DriveSubsystem driveSubsystem,CameraSubsystem cameraSubsystem) {
    m_cameraSubsystem = cameraSubsystem;
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cameraSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_cameraSubsystem.getNumContours() > 0){
      if(m_cameraSubsystem.getCenterX() > 320){
        m_driveSubsystem.driveRaw(-0.1, 0.1);

      }else if (m_cameraSubsystem.getCenterX() < 320){
        m_driveSubsystem.driveRaw(0.1, -0.1);
      }else{
        m_driveSubsystem.driveRaw(0,0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
