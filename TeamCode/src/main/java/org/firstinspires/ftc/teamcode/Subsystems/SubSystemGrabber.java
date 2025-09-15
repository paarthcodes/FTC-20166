package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemGrabber {
    // Instantiate the drivetrain motor variables
    private Servo grabberLeft;
    private Servo grabberRight;
    private RobotConstants robotConstants;

    /*
    Grabber servo subsystem.
    Subsystem has 2 servos mirrored.
    Open and close positions are set in robotConstants.
    @param hardwareMap Hardware map for the robot
    @param initState   Initial state of the servos when subsystem created`. 0 = uninitialized, 1 = open, 2 = closed
     */
    public SubSystemGrabber(HardwareMap hardwareMap, int initState) throws InterruptedException
    {
         grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
         grabberRight = hardwareMap.get(Servo.class, "grabberRight");
         //Only initialize the servo if erquested. The "do nothing" option hopefully stops the movement at teleop init
         if (initState != 0)
         {
             if (initState == 1)
                setPosition(robotConstants.GRABBER_OPEN_POSITION);
             else
                 setPosition(robotConstants.GRABBER_CLOSE_POSITION);
         }
     }

    public void setPosition(double position) {
        grabberLeft.setPosition(position);
        grabberRight.setPosition(1.0 - position);

    }

    public double getLeftPosition() {
        return grabberLeft.getPosition();
    }
    public double getRightPosition() {
        return grabberRight.getPosition();
    }

}