package frc.robot.subsystems;

import edu.wpi.first.cameraserver.*;

public class CameraSubsystem {
    public void startCapture() {
        CameraServer.startAutomaticCapture(); // Push this to the driverstation
    }
}
