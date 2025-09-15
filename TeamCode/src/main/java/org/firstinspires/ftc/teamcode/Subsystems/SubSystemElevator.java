package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.ELEVATOR_MULTIPLIER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class SubSystemElevator {
    // Instantiate the drivetrain motor variables
    private DcMotorEx elevator;
    private DcMotorEx elevator2;
    public double multiplier;
    private int targetPosition;
    private int numMotors = 0;




    public SubSystemElevator(HardwareMap hardwareMap, double multiplier, int motorCount) throws InterruptedException {                 // Motor Mapping
        // Initialize the motor hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");       //Sets the names of the hardware on the hardware map
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);

        numMotors = motorCount;

        if (numMotors == 2)
        {
            elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");       //Sets the names of the hardware on the hardware map
            elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
            elevator2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            elevator2.setTargetPosition(0);
            elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            elevator2.setPower(1);
        }
        this.multiplier = multiplier;
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
        targetPosition = (int)(encoderPosition * multiplier) ;
        elevator.setTargetPosition((int) (targetPosition));
        if (numMotors == 2)
        {
            elevator2.setTargetPosition((int) (targetPosition));
        }
    }

    public int getPosition() {
        return (elevator.getCurrentPosition());
    }

    public int getPosition2() {
        return (elevator2.getCurrentPosition());
    }

    public void setPower(double power) {
        elevator.setPower(power);
    }
    public void nudgeElevator(int nudge)
    {
        elevator.setTargetPosition(elevator.getCurrentPosition() - nudge);

    }
    public void resetElevator()
    {
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);
    }
}