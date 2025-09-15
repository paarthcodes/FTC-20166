package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp
//@Disabled
public class ControlSystemTest extends LinearOpMode {

    //Motor demo variables
    private DcMotorEx m0 = null;
    private DcMotorEx m1 = null;
    private DcMotorEx m2 = null;
    private DcMotorEx m3 = null;
    private DcMotorEx m4 = null;
    private DcMotorEx m5 = null;
    private DcMotorEx m6 = null;
    private DcMotorEx m7 = null;
    private IMU imu;

    public void initializeHardware()
    {
        m0 = hardwareMap.get(DcMotorEx.class, "FL");
        m1 = hardwareMap.get(DcMotorEx.class, "FR");
        m2 = hardwareMap.get(DcMotorEx.class, "BL");
        m3 = hardwareMap.get(DcMotorEx.class, "BR");
        m4 = hardwareMap.get(DcMotorEx.class, "M4");
        m5 = hardwareMap.get(DcMotorEx.class, "M5");
        m6 = hardwareMap.get(DcMotorEx.class, "M6");
        m7 = hardwareMap.get(DcMotorEx.class, "M7");

        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void runOpMode() throws InterruptedException {
       initializeHardware();

        waitForStart();
        while (opModeIsActive())
        {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Motor 0", m0.getCurrentPosition());
            telemetry.addData("Motor 1", m1.getCurrentPosition());
            telemetry.addData("Motor 2", m2.getCurrentPosition());
            telemetry.addData("Motor 3", m3.getCurrentPosition());
            telemetry.addData("Motor 4", m4.getCurrentPosition());
            telemetry.addData("Motor 5", m5.getCurrentPosition());
            telemetry.addData("Motor 6", m6.getCurrentPosition());
            telemetry.addData("Motor 7", m7.getCurrentPosition());
            telemetry.addData("IMU X", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("IMU Y", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("IMU Z", orientation.getRoll(AngleUnit.DEGREES));

            if(gamepad1.x)
                m0.setPower(-gamepad1.left_stick_y);
            else
                m0.setPower(0);

            if(gamepad1.y)
                m1.setPower(-gamepad1.left_stick_y);
            else
                m1.setPower(0);

            if(gamepad1.a)
                m2.setPower(-gamepad1.left_stick_y);
            else
                m2.setPower(0);

            if(gamepad1.b)
                m3.setPower(-gamepad1.left_stick_y);
            else
                m3.setPower(0);

            if(gamepad1.dpad_up)
                m4.setPower(-gamepad1.left_stick_y);
            else
                m4.setPower(0);

            if(gamepad1.dpad_left)
                m5.setPower(-gamepad1.left_stick_y);
            else
                m5.setPower(0);

            if(gamepad1.dpad_right)
                m6.setPower(-gamepad1.left_stick_y);
            else
                m6.setPower(0);

            if(gamepad1.dpad_down)
                m7.setPower(-gamepad1.left_stick_y);
            else
                m7.setPower(0);

            updateTelemetry(telemetry);
        }
    }
}




