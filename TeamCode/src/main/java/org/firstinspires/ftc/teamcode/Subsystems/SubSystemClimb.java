package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSystemClimb {
    // Instantiate the drivetrain motor variables
    private DcMotorEx climber;

    private int targetPosition;





    public SubSystemClimb(HardwareMap hardwareMap) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        climber = hardwareMap.get(DcMotorEx.class, "climber");       //Sets the names of the hardware on the hardware map
//        climber.setDirection(DcMotorSimple.Direction.REVERSE);
        climber.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climber.setTargetPosition(0);
        climber.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climber.setPower(1);
    }

    public boolean atTargetYet(int accuracy)
    {

        int error = Math.abs(targetPosition - getPosition());
        if (error > accuracy)
            return false;
        else
            return true;
    }

    public int getTarget()
    {
        return targetPosition;
    }
    public void setPosition(int encoderPosition)
    {
        targetPosition = (int)(encoderPosition) ;
        climber.setTargetPosition((int) (targetPosition));
    }

    public int getPosition() {
        return (climber.getCurrentPosition());
    }

    public void setPower(double power) {
        climber.setPower(power);
    }

}