package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.SubSystemClimb;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;


@TeleOp
//@Disabled
public class MotorTest extends LinearOpMode {

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx climber = null;
    private SubSystemGrabber robotGrabber = null;
    private SubSystemIntakeSlide robotIntakeSlide = null;

    private SubSystemIntakeArm robotIntakeArm = null;
    private SubSystemRobotID robotRobotID = null;
    private static final double GRABBER_OPEN_POSITION = 0.7;
    private static final double GRABBER_CLOSE_POSITION = 0.95;
    private int robotID = 0;





    public void initializeDriveMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
private void initalizeGrabber() throws InterruptedException
{
    robotGrabber = new SubSystemGrabber(hardwareMap, 0);
    robotGrabber.setPosition(GRABBER_OPEN_POSITION);
}
    private void setGrabberServo (boolean state)
    {
        if (state)
            robotGrabber.setPosition(GRABBER_OPEN_POSITION);
        else
            robotGrabber.setPosition(GRABBER_CLOSE_POSITION);
    }

    private void initalizeIntakeSlide() throws InterruptedException
    {
        robotIntakeSlide = new SubSystemIntakeSlide(hardwareMap);

    }
 private void initalizeIntakePivot() throws InterruptedException {
     robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
 }

 private void initializeRobotID() throws InterruptedException {
     robotRobotID = new SubSystemRobotID(hardwareMap);
     robotID = robotRobotID.getRobotID();
 }

 private void initializeClimb() throws InterruptedException
 {
     climber = hardwareMap.get(DcMotorEx.class, "climber");
     climber.setDirection(DcMotorSimple.Direction.REVERSE);
     climber.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
     climber.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
 }
    public void runOpMode() throws InterruptedException {
        initializeRobotID();
        initializeDriveMotors();
        initalizeGrabber();
        initalizeIntakeSlide();
        initalizeIntakePivot();
        initializeClimb();

        waitForStart();
        while (opModeIsActive())
        {
            robotID = robotRobotID.getRobotID();

            telemetry.addData("FL (0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            telemetry.addData("Robot ID", robotID);
            telemetry.addData("Encoder value", climber.getCurrentPosition());


            if(gamepad1.x)
                frontLeftDrive.setPower(-gamepad1.left_stick_y);
            else
                frontLeftDrive.setPower(0);

            if(gamepad1.y)
                frontRightDrive.setPower(-gamepad1.left_stick_y);
            else
                frontRightDrive.setPower(0);

            if(gamepad1.a)
                backLeftDrive.setPower(-gamepad1.left_stick_y);
            else
                backLeftDrive.setPower(0);

            if(gamepad1.b)
                backRightDrive.setPower(-gamepad1.left_stick_y);
            else
                backRightDrive.setPower(0);
            if (gamepad1.dpad_left)
                setGrabberServo(true);
            if (gamepad1.dpad_right)
                setGrabberServo(false);

            if (gamepad1.dpad_up)
            {
                robotIntakeArm.setPosition(0.1);
                telemetry.addData("Intake arm up)", robotIntakeArm.getPosition());

            }
            else if (gamepad1.dpad_down)
            {
                robotIntakeArm.setPosition(0.75);
                telemetry.addData("Intake arm down", robotIntakeArm.getPosition());
            }
            if (gamepad1.y)
            {
                climber.setPower(0.2);
            }
            else if (gamepad1.a)
            {
                climber.setPower(-0.2);
            }
            else {
                climber.setPower(0);
            }


            updateTelemetry(telemetry);
        }
    }
}




