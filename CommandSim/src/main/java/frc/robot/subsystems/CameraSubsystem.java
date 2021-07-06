// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GripPipeline;

public class CameraSubsystem extends SubsystemBase {
  
  private GripPipeline gripPipeline;
  private Thread visionThread;

  private boolean runProcessing = false;
  private double centerX;
  private double centerY;
  private int contoursFound;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    enableVisionThread();
  }

  public void enableVisionThread(){
    gripPipeline = new GripPipeline();
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    camera.setResolution(640, 480);


    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource outputStream = CameraServer.getInstance().putVideo("Stream", 640, 480);

    Mat mat = new Mat();

    runProcessing = true;
    contoursFound = 0;

    visionThread = new Thread(
      () -> {
        while (!Thread.interrupted()) {
          if(cvSink.grabFrame(mat) == 0){
            outputStream.notifyError(cvSink.getError());

            continue;
          }

          if(runProcessing){

            gripPipeline.process(mat);
            contoursFound = gripPipeline.convexHullsOutput().size();
            SmartDashboard.putString("More vision state", "Saw " + contoursFound + " Contours");

            if(contoursFound > 0){
              Rect rect = Imgproc.boundingRect(gripPipeline.convexHullsOutput().get(0));

              Imgproc.rectangle(mat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y+rect.height), new Scalar(0,0,255), 2);
              centerX = rect.x + rect.width/2;
              centerY = rect.y + rect.height/2;

              SmartDashboard.putString("Vision state", "Executed overlay!");
              SmartDashboard.putNumber("Center X", centerX);
            }

            outputStream.putFrame(mat);


            }

        }

      }
    );
    visionThread.setDaemon(true);
    visionThread.start();


  }


  public double getCenterX(){
    return centerX;
  }

  public void disableProcessing(){
    runProcessing = false;
  }

  public void enableProcessing(){
    runProcessing = true;
  }

  public int getNumContours(){
    return contoursFound;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
