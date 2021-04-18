// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class ArmSubsystem extends SubsystemBase {
  //Speed controller
  private final PWMSparkMax m_motor = new PWMSparkMax(4);

  //Encoder
  private final Encoder m_encoder = new Encoder(4, 5);

  //Gearbox 
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  //PID controller
  private final PIDController m_controller = new PIDController(3, 5, 0.1);

  //Feedforward controller 
  private final ArmFeedforward m_feedforward = new ArmFeedforward(100, 4, 1, 0);

  //Simulation stuff
  private final SingleJointedArmSim m_armSim = 
    new SingleJointedArmSim(
      m_armGearbox,
      100, 
      SingleJointedArmSim.estimateMOI(1, 6.0), 
      1, 
      Units.degreesToRadians(0), 
      Units.degreesToRadians(180), 
      6.0, 
      true,
      VecBuilder.fill(Units.degreesToRadians(0)));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);




  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Set the encoder pulse
    m_encoder.setDistancePerPulse(2*Math.PI/4096.0);
    m_encoder.reset();

    //Put the arm angle on SmartDashboard
    SmartDashboard.putNumber("Arm Angle", 0);
    
    //set the tolerance for the PID controller
    m_controller.setTolerance(0.001, 0.001);
  }


  @Override
  public void simulationPeriodic() {
    //Put the PID controller on Smart Dashboard
    SmartDashboard.putData("Arm PID Controller", m_controller);


    //Connect the motor to the arm simulator
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());  


    //Simulation stuff
    m_armSim.update(0.02);
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps())
    );

    //Update the arm angle value on SmartDashboard
    SmartDashboard.putNumber("Arm Angle", Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  //Set the arm motor to a speed
  public void setMotor(double pow){
    m_motor.set(pow);
  }


  //get the angle of the arm
  public double getAngle(){
    return Units.radiansToDegrees(m_armSim.getAngleRads());
  }


  //stop the motor
  public void stopMotor(){
    m_motor.stopMotor();
  }

  //Use PID to set the arm to an angle
  public void setArmAngle(double angle){
    var pidOutput = m_controller.calculate(getAngle(), angle);
    m_motor.setVoltage(pidOutput);
    // m_motor.setVoltage(m_feedforward.calculate(Units.degreesToRadians(45), 0));
    // m_motor.setVoltage(5);
  }

  //Reset the encoder
  public void resetEncoder(){
    m_encoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
