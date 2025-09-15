package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;

@Config
@TeleOp
//@Disabled
public class ClosedLoopMotorTest extends LinearOpMode {

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx climber = null;
    private Telemetry telemetryA;

    private static final double GRABBER_OPEN_POSITION = 0.7;
    private static final double GRABBER_CLOSE_POSITION = 0.95;
    private int robotID = 0;
    
    private int max_tps = 2400; // 2400 = 5100 rpm

    
    public double desired_Velocity;
    
    public double measured_Velocity;





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

    public void runOpMode() throws InterruptedException {
        initializeDriveMotors();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        waitForStart();
        while (opModeIsActive())
        {
            //robotID = robotRobotID.getRobotID();

            /*if(gamepad1.x)
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
            */
            
            desired_Velocity = max_tps * gamepad1.right_trigger;
            
            //if (gamepad1.right_bumper)
            {
                frontLeftDrive.setVelocity(desired_Velocity);
                frontRightDrive.setVelocity(desired_Velocity);
                backLeftDrive.setVelocity(desired_Velocity);
                backRightDrive.setVelocity(desired_Velocity);
            }

            if (gamepad1.a)
            {
                desired_Velocity = 1000;
            }

            measured_Velocity = frontLeftDrive.getVelocity();

            telemetryA.addData("Desired Velocity", desired_Velocity);
            telemetryA.addData("Measured Velocity", measured_Velocity);
            telemetryA.addData("Trigger", gamepad1.right_trigger);
            /*telemetryA.addData("FR (1)", frontRightDrive.getCurrentPosition());
            telemetryA.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetryA.addData("BR (3)", backRightDrive.getCurrentPosition());
            telemetryA.addData("Robot ID", robotID);
            telemetryA.addData("Encoder value", climber.getCurrentPosition());*/
            
            updateTelemetry(telemetryA);
        }
    }
}




