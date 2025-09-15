package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSystemIntake {
    // Instantiate the drivetrain motor variables

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private CRServo topIntakeServo;


    public SubSystemIntake(HardwareMap hardwareMap) throws InterruptedException {
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        topIntakeServo = hardwareMap.get(CRServo.class, "topIntakeServo");
     }

    public void setSpeed(double speed)
    {
        leftIntakeServo.setPower(-speed);
        rightIntakeServo.setPower(speed);
        topIntakeServo.setPower(speed);
    }

}