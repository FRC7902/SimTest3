// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardAndTurn extends SequentialCommandGroup {
  /** Creates a new DriveForwardAndTurn. */
  public DriveForwardAndTurn(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      
      new DriveToDistance(9, driveSubsystem), // Go forward 9 meters
      new TurnToAngle(-90, driveSubsystem), // Turn 90 deg clockwise
      new DriveToDistance(11, driveSubsystem), //Go forward 2 meters
      new TurnToAngle(180, driveSubsystem), // Turn 90 deg clockwise
      new DriveToDistance(20, driveSubsystem) //Go forward 9 meters



    );
  }
}
