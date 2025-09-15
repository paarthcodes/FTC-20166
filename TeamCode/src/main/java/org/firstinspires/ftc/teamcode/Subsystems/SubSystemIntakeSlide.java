package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemIntakeSlide
{
    private Servo intakeSlideServo;

    private double targetServoPosition = 0;

    private double currentServoPosition = 0;

    private double sliderSpeed = 0.03;


    public SubSystemIntakeSlide(HardwareMap hardwareMap) throws InterruptedException {

        intakeSlideServo = hardwareMap.get(Servo.class, "intakeSlideServo");
    }


    public void setPosition(double position)
    {
        if (position < 0)
        {
            position = 0;
        }
        else if (position > 1)
        {
            position = 1;
        }
        targetServoPosition = RobotConstants.INTAKE_SLIDE_MIN_POSITION +((RobotConstants.INTAKE_SLIDE_MAX_POSITION - RobotConstants.INTAKE_SLIDE_MIN_POSITION) * position);
    }

    public void setPositionNow(double position)
    {
        if (position < 0)
        {
            position = 0;
        }
        else if (position > 1)
        {
            position = 1;
        }
        currentServoPosition = position;
        targetServoPosition = position;

        intakeSlideServo.setPosition(1.0-currentServoPosition);

    }
    public void updateServoPosition()
    {
        if (targetServoPosition > currentServoPosition)
        {
            currentServoPosition = currentServoPosition + sliderSpeed;
            if (currentServoPosition > targetServoPosition)
            {
                currentServoPosition = targetServoPosition;
            }
        }
        else if (targetServoPosition < currentServoPosition)
        {
            currentServoPosition = currentServoPosition - sliderSpeed;
            if (currentServoPosition < targetServoPosition)
            {
                currentServoPosition = targetServoPosition;
            }
        }

        intakeSlideServo.setPosition(1.0-currentServoPosition);

    }

}
