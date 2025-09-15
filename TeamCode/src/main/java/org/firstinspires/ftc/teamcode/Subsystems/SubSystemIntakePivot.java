package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSystemIntakePivot {
    private Servo intakePivotServo;




    public SubSystemIntakePivot(HardwareMap hardwareMap) throws InterruptedException {
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");
    }


    public void setPosition(double position)
    {


        intakePivotServo.setPosition(position);
    }





    public double getPosition()
    {
        return intakePivotServo.getPosition();
    }
}
