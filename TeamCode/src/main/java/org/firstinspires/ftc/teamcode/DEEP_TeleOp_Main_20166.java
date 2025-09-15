package org.firstinspires.ftc.teamcode;

//config name                hub                slot                    description
// Fl                        control            Motor 0                 frontLeft Drive motor
// FR                        control            Motor 1                 frontRight Drive motor
// BL                        control            Motor 2                 backLeft Drive motor
// BR                        control            Motor 3                 backRight Drive motor

// elevator                  expansion          motor 0                 elevator motor
// elevator2                 expansion          motor 1                 elevator motor 2

// grabberLeft               control            servo 0                 grabberLeft servo
// grabberRight              control            servo 1                 grabberRight servo
// intakePivotServo          control            servo 2                 intakePivot servo
// intakeSlideServo          control            servo 3                 intakeSlide servo
// leftIntakeArmServo        control            servo 4                 leftIntakeArmServo servo
// rightIntakeArmServo       control            servo 5                 rightIntakeArmServo servo

// blinkIn                   expansion          servo 1                 ledDriver
// leftIntakeServo           expansion          servo 2                 leftIntakeServo
// rightIntakeServo          expansion          servo 3                 rightIntakeServo
// topIntakeServo            expansion          servo 4                 topIntakeServo

//OTOS                       control            i2bus 1                 OTOS sensor
// colorSensor               control            i2Bus 2                 color sensor
// limitSwitch               control            digital 0               limitSwitch digital channel
// limitSwitchTwo            control            digital 1               limitSwitchTwo digital channel




import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemClimb;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntake;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakePivot;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class DEEP_TeleOp_Main_20166 extends LinearOpMode {
    SparkFunOTOS myOtos;

    ElapsedTime timer = new ElapsedTime();
    private RobotConstants robotConstants;
    private int elevatorMoveTo = 0;
    private double translateX;
    private double translateY;
    private double joy1RightX;
    private double joy1RightY;//
    private double FLMP, FRMP, BLMP, BRMP;

    private boolean elevatorMoveBottom = false;
    private boolean elevatorMoveTop = false;
    private boolean elevatorMoveLow = false;
    private boolean elevatorMoveHigh = false;
    private boolean elevatorMode = false;

    private boolean grabberOpen;
    private boolean grabberClose;
    NormalizedColorSensor colorSensor;


    private double intakeSpeed = 0;
    private double intakeArmPosition = 0;

    private double intakePivotPosition = 0;
    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
//    private IMU imu         = null;
    private Servo intakePivot;

    private double intakeSlidePosition = 0;

    private SubSystemRobotID robotRobotID = null;

    private SubSystemElevator robotElevator = null;
    private SubSystemGrabber robotGrabber = null;
    private SubSystemIntakeArm robotIntakeArm = null;
    private SubSystemIntake robotIntake = null;
    private SubSystemIntakeSlide robotIntakeSlide = null;
    private SubSystemIntakePivot robotIntakePivot = null;

    private SubSystemClimb robotClimber = null;



    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    private boolean driverAssistPickup = false;
    private float intakeIn;
    private float intakeOut;
    private boolean spatulaMoveDown = false;
    public boolean intakeOverride = false;

    public int climberCurrentPosition;
    public int climberDirection = 0;

    private int robotID = 0;

    public void initializeDriveMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        if (robotID == 2)
        {
            frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeSensors()
    {

        //Initialize gyro etc...
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();
        //rev magnet thing

        //Initialize the color sensor

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");




    }

    private void initializeSubSystems() throws InterruptedException {

        robotRobotID = new SubSystemRobotID(hardwareMap);
        robotID = robotRobotID.getRobotID();
        robotConstants = new RobotConstants(robotID);

        if (robotID == 2)
            robotElevator = new SubSystemElevator(hardwareMap, robotConstants.ELEVATOR_MULTIPLIER, 2);
        else
            robotElevator = new SubSystemElevator(hardwareMap, robotConstants.ELEVATOR_MULTIPLIER, 1);
        robotGrabber = new SubSystemGrabber(hardwareMap, 0);
        robotIntake = new SubSystemIntake(hardwareMap);
        robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
        robotIntakeSlide = new SubSystemIntakeSlide(hardwareMap);
        robotIntakePivot = new SubSystemIntakePivot(hardwareMap);
        robotClimber = new SubSystemClimb(hardwareMap);
//        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        //intakePivot.setPosition(0.5);
    }

    private void initializeLEDs(){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    private void initalizeEverything() throws InterruptedException {
        initializeSensors();
        initializeSubSystems();
        initializeDriveMotors();
        initializeLEDs();
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    }

    public String getSampleColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        float maxsat = Math.max(Math.max(colors.red, colors.green), colors.blue);
        float r = colors.red/maxsat;
        float g = colors.green/maxsat;
        float b = colors.blue/maxsat;
        String sample = "Nothing";

        if (timer.time() > 100)
        {
            sample = "Endgame";
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER);
        }
        else {

            if (distance < 8.0) {
                if (r == 1.0) {
                    sample = "Red";
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else if (b == 1.00) {
                    sample = "Blue";
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                } else {
                    sample = "Yellow";
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
            } else {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

            }
        }
            return sample;

    }

    public double getHeadingRadians() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //return orientation.getYaw(AngleUnit.RADIANS);
        return Math.toRadians(getHeadingDegrees());
    }

    public double getHeadingDegrees() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
       // return orientation.getYaw(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return pos.h;
    }

    private static double[] rotatePoint(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        Result[0] = xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
        Result[1] = xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
        return Result;
    }

    private static double rotatePointX(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
    }

    private static double rotatePointY(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
    }

    private void updateJoysticks()
    {
        //GAMEPAD1:
        //dpad up - Moving intake arm up
        //dpad down - Moving intake arm down
        //dpad left -
        //dpad right -
        //a -
        //b -
        //y -
        //x - Driver Assist
        //left bumper (lb) -
        //right bumper (rb) - Switch to robot centric
        //left trigger (lt) -
        //right trigger (rt) -
        //left joystick - Translate robot
        //right joystick x - Rotating robot
        //right joystick y -
        //GAMEPAD2:
        //dpad up - elevatorMoveTop
        //dpad down - elevatorMoveBottom
        //dpad left - elevatorMoveLow
        //dpad right - elevatorMoveHigh
        //a - intake in
        //b - grabberClose
        //y - intake out
        //x - grabberOpen
        //left bumper (lb) -
        //right bumper (rb) - intake arm (pressed = down, not pressed = up) also controls intake pivot
        //left trigger (lt) -
        //right trigger (rt) -
        //left joystick -
        //right joystick -
        //driving
        double x, y, heading;
        heading = getHeadingRadians();

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        joy1RightX = gamepad1.right_stick_x;
        joy1RightY = gamepad1.right_stick_y;

        if (gamepad1.right_bumper){//Switch to robot centric
            translateX = x;
            translateY = y;
        }
        else{
            translateX = rotatePointX(x, y, heading);
            translateY = rotatePointY(x, y, heading);
        }

        elevatorMoveBottom = gamepad2.dpad_down;
        elevatorMoveTop = gamepad2.dpad_up;
        elevatorMoveLow = gamepad2.dpad_left;
        elevatorMoveHigh = gamepad2.dpad_right;
        elevatorMode = gamepad2.left_bumper;

        grabberOpen = gamepad2.x;
        grabberClose = gamepad2.b;

        //elevatorNetSpeed = gamepad1.right_trigger - gamepad1.left_trigger;

        if (gamepad2.y)
            intakeSpeed = robotConstants.INTAKE_ROLLER_OUT_SPEED;
        else if (gamepad2.a)
            intakeSpeed = robotConstants.INTAKE_ROLLER_IN_SPEED;
        else
            intakeSpeed = robotConstants.INTAKE_ROLLER_HOLD_SPEED;

//        if (gamepad2.y) if the arm is not completely down to drop specimen, the intake rollers will not drop it
        // if (targetArmServoPosition == currentArmServoPosition)
//        {
//            intakeSpeed = robotConstants.INTAKE_ROLLER_OUT_SPEED;
//        }

        //buttons
//        if(gamepad1.guide && gamepad2.guide) {
//            imu.resetYaw();
//        }

        intakeSlidePosition = -gamepad2.left_stick_y;

        //Spatula
        spatulaMoveDown = gamepad2.right_bumper;

        //Climb

        if (gamepad1.y)
        {
            climberDirection = 1;
        }
        else if (gamepad1.a)
        {
            climberDirection = -1;
        }
        else
        {
            climberDirection = 0;
        }



    }

     private void updateIntakeArm()
    {
        if (spatulaMoveDown)
        {
            intakeArmPosition = robotConstants.INTAKE_ARM_DOWN_POSITION;
            intakePivotPosition = robotConstants.INTAKE_PIVOT_PICKUP_POSITION;
            intakeOverride = true;

        }
        /*else if (Math.abs(robotElevator.getPosition() - (robotConstants.ELEVATOR_TRANSFER_SAMPLE_POSITION * robotElevator.multiplier)) < 5)
        {
            intakeArmPosition = robotConstants.INTAKE_ARM_TRANSFER_SAMPLE_POSITION;

        }*/
        else {
            intakeArmPosition = robotConstants.INTAKE_ARM_UP_POSITION;
            intakePivotPosition = robotConstants.INTAKE_PIVOT_IDLE_POSITION;
            intakeOverride = false;
        }
        robotIntakeArm.setPosition(intakeArmPosition);

        robotIntakePivot.setPosition(intakePivotPosition);
        robotIntakeArm.updateArmPosition();

    }


    private void updateIntakeSlide()
    {
        if (Math.abs(robotElevator.getPosition()) > 50)
        {
            robotIntakeSlide.setPosition(robotConstants.INTAKE_SLIDE_SAFE_POSITION);
        }
        else
        {
            robotIntakeSlide.setPosition(intakeSlidePosition);
        }

        robotIntakeSlide.updateServoPosition();
    }

    private void updateClimber()
    {
        climberCurrentPosition = robotClimber.getPosition();

        if ((climberDirection == 1) && (climberCurrentPosition < robotConstants.CLIMBER_UP_POSITION))
        {
            robotClimber.setPosition(climberCurrentPosition + robotConstants.CLIMBER_SPEED);
        }
        //else if ((climberDirection == -1) && (climberCurrentPosition > 0))
        else if ((climberDirection == -1) && (climberCurrentPosition > robotConstants.CLIMBER_HANG_POSITION))
        {
            robotClimber.setPosition(climberCurrentPosition - robotConstants.CLIMBER_SPEED);
        }


    }
    private void updateDashboard()
    {
//        telemetry.addData("FL",frontLeftDrive.getCurrentPosition());
//        telemetry.addData("FR",frontRightDrive.getCurrentPosition());
//        telemetry.addData("BL",backLeftDrive.getCurrentPosition());
//        telemetry.addData("BR",backRightDrive.getCurrentPosition());
        telemetry.addData("Heading", getHeadingDegrees());

        telemetry.addLine("\n");
        telemetry.addData("Elevator Pos", robotElevator.getPosition());
        telemetry.addData("Elevator moveto", elevatorMoveTo);

        telemetry.addLine("\n");
        telemetry.addData("Sample detected", getSampleColor());

        telemetry.addData("Climb position", climberCurrentPosition);
        telemetry.addData("Climb direction", climberDirection);

/*
        telemetry.addLine("\n");
        telemetry.addData("Left Grabber Servo Position: ", robotGrabber.getLeftPosition());
        telemetry.addData("Right Grabber Servo Position: ", robotGrabber.getRightPosition());

        telemetry.addLine("\n");
        telemetry.addData("Spatula moving down?: ", spatulaMoveDown);
        telemetry.addData("Spatula position: ", robotIntakeArm.getPosition());

        telemetry.addLine("\n");
        telemetry.addData("Gamepad Guide gamepad 1: ", gamepad1.guide);
        telemetry.addData("Gamepad Guide gamepad 2: ", gamepad2.guide);

        telemetry.addLine("\n");
 */
        telemetry.addData("Robot ID: ", robotID);
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);

        updateTelemetry(telemetry);
    }
    private void updateDrivebaseMotors(double FLMP, double FRMP, double BLMP, double BRMP)
    {
        double maxPower;

        maxPower = Math.max(Math.max(FLMP, FRMP), Math.max(BLMP, BRMP));

        if (maxPower > 1.0)
        {
            FLMP = FLMP / maxPower;
            FRMP = FRMP / maxPower;
            BLMP = BLMP / maxPower;
            BRMP = BRMP / maxPower;
        }
        frontLeftDrive.setPower(FLMP);
        frontRightDrive.setPower(FRMP);
        backLeftDrive.setPower(BLMP);
        backRightDrive.setPower(BRMP);
    }
    private void calculateDrivebaseSpeed()
    {
        double FLPFB, FRPFB, BLPFB, BRPFB;
        double FLPLR, FRPLR, BLPLR, BRPLR;
        double FLPR, FRPR, BLPR, BRPR;

        FLPFB = translateY;
        FRPFB = translateY;
        BLPFB = translateY;
        BRPFB = translateY;

        FLPLR = -translateX;
        FRPLR = translateX;
        BLPLR = translateX;
        BRPLR = -translateX;

        FLPR = -joy1RightX;
        FRPR = joy1RightX;
        BLPR = -joy1RightX;
        BRPR = joy1RightX;

        FLMP = FLPFB + FLPLR + FLPR;
        FRMP = FRPFB + FRPLR + FRPR;
        BLMP = BLPFB + BLPLR + BLPR;
        BRMP = BRPFB + BRPLR + BRPR;
    }
    private void setElevator(int position)
    {
        robotElevator.setPosition(position);
        elevatorMoveTo = position;
    }
    public void updateElevator()
    {
        if (gamepad1.right_trigger > 0.5)
        {
            setElevator(robotElevator.getPosition() + (int)(25*robotConstants.ELEVATOR_MULTIPLIER));
        }
        else if (gamepad1.left_trigger > 0.5)
        {
            setElevator(robotElevator.getPosition() - (int)(25*robotConstants.ELEVATOR_MULTIPLIER));
        }

        if (elevatorMode) //Basket mode
        {
            if (elevatorMoveBottom == true) {
                setElevator(robotConstants.ELEVATOR_BOTTOM_POSITION);
            } else if (elevatorMoveTop == true) {
                setElevator(robotConstants.ELEVATOR_TOP_BASKET);
            } else if (elevatorMoveLow == true) {
                setElevator(robotConstants.ELEVATOR_LOW_BASKET);
            } else if (elevatorMoveHigh == true) {
                setElevator(robotConstants.ELEVATOR_TRANSFER_SAMPLE_POSITION);
            }
        }
        else //Observation mode
        {
            if (elevatorMoveBottom == true) {
                setElevator(robotConstants.ELEVATOR_BOTTOM_POSITION);
            } else if (elevatorMoveTop == true) {
                setElevator(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
            } else if (elevatorMoveLow == true) {
                setElevator(robotConstants.ELEVATOR_SPECIMEN_PICKUP);
            } else if (elevatorMoveHigh == true) {
                setElevator(robotConstants.ELEVATOR_TOP_RUNG_RELEASE);
            }
        }

        if (gamepad1.right_bumper && gamepad1.start)
        {
            robotElevator.nudgeElevator(50);
        }
        if (gamepad1.left_bumper && gamepad1.start)
        {
            robotElevator.resetElevator();
        }

        /*if ((elevatorNetSpeed > 0.1) && (robotElevator.getPosition() < robotConstants.ELEVATOR_TOP_BASKET * robotConstants.ELEVATOR_MULTIPLIER)){
            robotElevator.setPower(elevatorNetSpeed);
            robotElevator.setPosition(robotElevator.getPosition() + 150);
        }
        else if ((elevatorNetSpeed < -0.1) && (robotElevator.getPosition() > robotConstants.ELEVATOR_BOTTOM_POSITION * robotConstants.ELEVATOR_MULTIPLIER)){
            robotElevator.setPower(elevatorNetSpeed);
            robotElevator.setPosition(robotElevator.getPosition() - 150);
        }
        else {
            robotElevator.setPower(1);//Hold the current position at half power
        }*/
    }
    private void setGrabberServo (boolean state)
    {
        if (state)
            robotGrabber.setPosition(robotConstants.GRABBER_OPEN_POSITION);
        else
            robotGrabber.setPosition(robotConstants.GRABBER_CLOSE_POSITION);
    }
    public void updateGrabber()
    {
       if (grabberOpen == true)
       {
           setGrabberServo(true);
       }
       else if (grabberClose == true)
        {
            setGrabberServo(false);
        }
    }

    private boolean rotateRobotToHeading(double heading)
    {
        double rotateSpeed = 0.45;
        double tolerance = 3;
        double headingError = getHeadingDegrees() - heading;
        if (headingError < -180)
            headingError = headingError + 360;
        else if (headingError > 180)
            headingError = headingError - 360;
        if (headingError < -tolerance)
        {
            updateDrivebaseMotors(rotateSpeed, -rotateSpeed, rotateSpeed,-rotateSpeed);
            return false;
        }
        else if (headingError > tolerance)
        {
            updateDrivebaseMotors(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
            return false;
        }
        else
        {
            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
        }
    }
    private boolean driveRobotToDistanceFrom (double distance)
    {
//        double currentDistance = frontDistanceSensor.getDistance(DistanceUnit.MM);
//        double driveSpeed = -0.25;
//        if (currentDistance > distance)
//        {
//            updateDrivebaseMotors(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
//            return false;
//
//        }
//            else
//        {
//            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
//        }
    }
    private DRIVER_ASSIST_STATE currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    private DRIVER_ASSIST_STATE returnDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    private enum DRIVER_ASSIST_STATE {STATE_WAIT, STATE_FACE_WALL, STATE_DRIVE_TO_WALL_1, STATE_WAIT_RELEASE, STATE_STRAFE_TO_WALL, STATE_CLOSE_AND_TURN};
    private void processStateWait()
    {
        if (driverAssistPickup)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_FACE_WALL;
        }
    }
    private void processStateFaceWall()
    {
        boolean done = rotateRobotToHeading(180.0);
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_DRIVE_TO_WALL_1;

    }
    private void processStateDriveToWall()
    {
        boolean done = driveRobotToDistanceFrom(120.0);
        if (done)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_STRAFE_TO_WALL;
            setGrabberServo(true);
            setElevator(robotConstants.ELEVATOR_SPECIMEN_PICKUP);
        }
    }
    private void processStateWaitRelease()
    {
        if (!driverAssistPickup)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    }
    private void processStateToWall()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateCloseAndTurn()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateStrafeToWall()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateDelay()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = returnDriverAssistState;
    }





    private void processStateMachine()
    {
        boolean done = false;
        if (!driverAssistPickup)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;

        }
        else
        {
            switch (currentDriverAssistState) {
                case STATE_WAIT:
                    processStateWait();
                    break;
                case STATE_FACE_WALL:
                    processStateFaceWall();
                    break;
                case STATE_DRIVE_TO_WALL_1:
                    processStateDriveToWall();
                    break;
                case STATE_STRAFE_TO_WALL:
                    processStateStrafeToWall();
                    break;
                case STATE_CLOSE_AND_TURN:
                    processStateCloseAndTurn();
                    break;
                case STATE_WAIT_RELEASE:
                    processStateWaitRelease();
                    break;
            }

        }


    }

    private void updateIntake()
    {
        if (timer.time() > 115)
            robotIntake.setSpeed(0);
        else if (intakeSpeed == robotConstants.INTAKE_ROLLER_OUT_SPEED)
            robotIntake.setSpeed(intakeSpeed);
        else if (intakeOverride)
            robotIntake.setSpeed(robotConstants.INTAKE_ROLLER_IN_SPEED);
        else
            robotIntake.setSpeed(intakeSpeed);

        telemetry.addData("intakeServo", intakeSpeed);

    }


    public void runOpMode() throws InterruptedException {
        initalizeEverything();

        waitForStart();
        timer.reset();
        while (opModeIsActive())
        {
            updateJoysticks();
            calculateDrivebaseSpeed();
            if (currentDriverAssistState == DRIVER_ASSIST_STATE.STATE_WAIT)
            {
                updateDrivebaseMotors(FLMP, FRMP, BLMP, BRMP);
                updateElevator();
                updateGrabber();
                updateIntake();
                updateIntakeArm();
                updateIntakeSlide();
                updateClimber();
            }
            processStateMachine();
            getSampleColor();
            updateDashboard();
        }

    }
}




