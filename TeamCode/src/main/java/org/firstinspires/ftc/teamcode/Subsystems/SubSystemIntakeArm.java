package org.firstinspires.ftc.teamcode.Subsystems;
//elevator
//

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemIntakeArm {
    // Instantiate the drivetrain motor variables
    private Servo leftServo;
    private Servo rightServo;
    private double targetArmServoPosition = 0;
    private double currentArmServoPosition = 0;

    private double armSpeed = 0.0315;

    public SubSystemIntakeArm(HardwareMap hardwareMap) throws InterruptedException {
        leftServo = hardwareMap.get(Servo.class, "leftIntakeArmServo");
        rightServo = hardwareMap.get(Servo.class, "rightIntakeArmServo");
    }

    public void setPosition(double position) {
        targetArmServoPosition = position;
    }

    public void setPositionNow(double position) {
        currentArmServoPosition = position;
        targetArmServoPosition = position;
        rightServo.setPosition(1 - currentArmServoPosition);
        leftServo.setPosition(currentArmServoPosition);
    }

    public void updateArmPosition() {
        if (targetArmServoPosition > currentArmServoPosition) {
            currentArmServoPosition = currentArmServoPosition + armSpeed;
            if (currentArmServoPosition > targetArmServoPosition) {
                currentArmServoPosition = targetArmServoPosition;
            }
        } else if (targetArmServoPosition < currentArmServoPosition) {
            currentArmServoPosition = currentArmServoPosition - armSpeed;
            if (currentArmServoPosition < targetArmServoPosition) {
                currentArmServoPosition = targetArmServoPosition;
            }
        }
        rightServo.setPosition(1 - currentArmServoPosition);
        leftServo.setPosition(currentArmServoPosition);

    }

    public double getPosition()
    {
        return rightServo.getPosition();
    }



}
