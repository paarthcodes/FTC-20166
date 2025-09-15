package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeft;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseLeftTwo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
/*
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickup;
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickupHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2Heading;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOff;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOffHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersible;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersibleHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1Heading;
//import static org.firstinspires.ftc.teamcode.wayPoints.*;
*/

/**
 * Testing autonomous using Pedro based on curved back and forth example
 *
 */
@Config
@Autonomous
@Disabled
public class LeftAutotestTwo20166 extends OpMode {
    private RobotConstants robotConstants;
    private Telemetry telemetryA;
    private SubSystemGrabber robotGrabber;
    private SubSystemElevator robotElevator;

    public double botHeading = startingPoseLeft.getHeading();
    private Follower follower;
    private SubSystemIntakeArm robotIntakeArm;
    private int robotID;
    private SubSystemRobotID robotRobotID = null;


    public double testX = 0;
    public double testY = 0;
    public double testHeading = 0;

    private enum AUTON_STATE
    {
        AUTON_START_STATE,
        LOWER_ELEVATOR_STATE,
        LOWER_ELEVATOR_SETUP_STATE,
        WAIT_AUTO_FINISHED,
        WAIT_PATH_DONE_STATE,
        PUSH_SAMPLES_STATE,
        MOVE_TO_SAMPLE_FROM_OBSERVATION_ZONE_STATE,
        MOVE_FROM_PICKUP_TO_SUBMERSIBLE_STATE,

        PICKUP_SPECIMEN_STATE,
        DO_NOTHING,
        MOVE_ELEVATOR_UP_STATE, DROPOFF_SPECIMEN_STATE
    }

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitPathDoneNextState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE lowerElevatorNextState = AUTON_STATE.AUTON_START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;
private static final int xOffset = 6;
    public static final Point startPoint = new Point (startingPoseLeftTwo.getX(), startingPoseLeftTwo.getY(), Point.CARTESIAN);
    //public static final Point submersibleDropPoint = new Point(-2.4+xOffset, -32.6, Point.CARTESIAN);

    //New Points for submersible to spikes ready to push into human player area
    public static final Pose sampleToBasketPreMove = pointAndHeadingToPose(-48, -54, 225);
    public static final Point submersibleToSpike2 = new Point(-56,-60 , Point.CARTESIAN);//elevator on top of basket, ready to drop sample one
    //grabber opens and drops sample
    public static final Point submersibleToSpike3 = new Point(-50,-51 , Point.CARTESIAN);//back up after dropping sample one in basket
    //robot will turn
    public static final Point submersibleToSpike4 = new Point(-49,-33 , Point.CARTESIAN);//collect sample two
    public static final Point submersibleToSpike5 = new Point(-50, -51,Point.CARTESIAN);//align to drop sample two
    //grabber opens to drop sample two
    public static final Point submersibleToSpike6 = new Point(-56, -60, Point.CARTESIAN); //elevator ready to drop
    //grabber opens
    public static final Point submersibleToSpike7 = new Point(-50, -51 , Point.CARTESIAN);//elevator backs out after dropping
    public static final Point submersibleToSpike8 = new Point(-58, -33, Point.CARTESIAN);//collect sample three
    public static final Point submersibleToSpike9 = new Point(-50, -51, Point.CARTESIAN);//align  to elevator basket
    public static final Point submersibleToSpike10 = new Point(-56, -60, Point.CARTESIAN);//elevator on top of basket, ready to drop sample one
    public static final Point submersibleToSpike11 = new Point(-50, -51, Point.CARTESIAN);//elevator backs out after dropping
    public static final Point submersibleToSpike12 = new Point(-65  , -33, Point.CARTESIAN);//collect sample four
    public static final Point submersibleToSpike13 = new Point(-50, -51,  Point.CARTESIAN);//align to drop sample four
    public static final Point submersibleToSpike14 = new Point( -56, -60, Point.CARTESIAN); //elevator ready to drop
    //grabber opens to drop sample four

    public static final Point submersibleToSpike15 = new Point(-50, -51, Point.CARTESIAN); //elevator backs out after dropping sample four


    public Path driveToSpecimenPickUp;
    public Point pickUpStartPoint = new Point(49.5,-55  , Point.CARTESIAN); //GET POINT
    public Point pickUpEndPoint = new Point(49.5,-59  , Point.CARTESIAN);
    public Point submersibleDepositPoint; //This is the drop off point for the specimens after the initial specimen.


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void start() {
        restartTimeout(robotConstants.LEFT_AUTON_DELAY);
    }
    @Override
    public void init()
    {
        try {
            robotRobotID = new SubSystemRobotID(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotID = robotRobotID.getRobotID();
        robotConstants = new RobotConstants(robotID);

        follower = new Follower(hardwareMap, robotConstants.DRIVE_DIRECTION);
        follower.setStartingPose(startingPoseLeftTwo);
        follower.update();

        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower.setMaxPower(robotConstants.START_TO_SUBMERSIBLE_SPEED);

        //initalizePathHeadings();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }

    private void initializeSubSystems() throws InterruptedException {

        robotElevator = new SubSystemElevator(hardwareMap, robotConstants.ELEVATOR_MULTIPLIER, robotConstants.ELEVATOR_MOTOR_COUNT);
        robotGrabber = new SubSystemGrabber(hardwareMap, 1);
        robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */


    //Close gripper
    //Lift elevator
    //Go to submersible
    //Attach (lower elevator) specimen onto high rung
    //Open claw
    //Go to spikes
    //Lower elevator
    //Open gripper
    //Go to sample 1
    //Pick up sample 1
    //Go to observation
    //Drop sample
    //Grab sample 2 in the observation area
    //Lift elevator to correct position
    //Go to submersible
    //Lower elevator down so the specimen will hang on the high rung.
    //Repeat until tele-op starts

    private void setupPath(Path pathToFollow, double endHeading)
    {
        double currentHeading = botHeading;
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
    }


    private void restartTimeout(int timeout)
    {
        timeoutPeriod = timeout;
        timeoutTimer.reset();
    }

    private boolean pathIsBusy()
    {
        //if (hasTimededout())
          //  return false;
        //else if (follower.isBusy())
          //  return true;
        //else
          //  return false;
        return follower.isBusy();
    }

    private boolean hasTimededout()
    {
        if (timeoutTimer.time() < timeoutPeriod)
            return false;
        else
            return true;
    }

    private void processLowerElevatorSetupState()
    {
        robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_RELEASE);
        restartTimeout(1000);
        currentAutonomousState = AUTON_STATE.LOWER_ELEVATOR_STATE;
    }
    private void processLowerElevatorState()
    {
        if((Math.abs(robotElevator.getPosition() - robotConstants.ELEVATOR_TOP_RUNG_RELEASE) < 5) || hasTimededout())
        {
            robotGrabber.setPosition(robotConstants.GRABBER_OPEN_POSITION);
            currentAutonomousState = lowerElevatorNextState;
            robotElevator.setPosition(robotConstants.ELEVATOR_SPECIMEN_PICKUP);
            follower.setMaxPower(robotConstants.SUBMERSIBLE_TO_PUSH_SPEED);
        }
    }


    private void processMoveToSampleFromObservationZoneState()
    {
        driveToSpecimenPickUp = new Path(new BezierCurve(pickUpStartPoint, pickUpEndPoint));
        setupPath(driveToSpecimenPickUp, 270);
        robotGrabber.setPosition(RobotConstants.GRABBER_CLOSE_POSITION);
        //robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
        robotElevator.setPosition(robotConstants.ELEVATOR_BOTTOM_POSITION);
    }

    private void processMoveFromPickupToSubmersibleState()
    {

    }


/////////////////////////////////////////////////////////////////
private Point poseToPoint(Pose pose)
{
    Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
    return returnPoint;
}
private static Pose pointAndHeadingToPose(double x, double y, double headingInDegrees)
{
    return new Pose(x, y, Math.toRadians(headingInDegrees));
}
    private void setPathFromCurrentPositionToTargetPose(Pose targetPose)
    {
        Point targetPoint = poseToPoint(targetPose);
        double targetHeading = Math.toDegrees(targetPose.getHeading());
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), targetPoint)), targetHeading);
        testX = targetPose.getX();
        testY = targetPose.getY();
        testHeading = targetHeading;
    }
    private void processWaitPathDone()
    {
        if (!pathIsBusy())
            currentAutonomousState = waitPathDoneNextState;
    }
    private void processStateStart()
    {
        if (hasTimededout())
        {
            setPathFromCurrentPositionToTargetPose(sampleToBasketPreMove);
            restartTimeout(20000);
            currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
            waitPathDoneNextState = AUTON_STATE.DO_NOTHING;
        }
    }
    private void processElevatorUpState()
    {

        robotElevator.setPosition(robotConstants.ELEVATOR_TOP_BASKET);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.DO_NOTHING;


    }
    private void processWaitAutonDoneState()
    {
        follower.holdPoint(new BezierPoint(submersibleToSpike13), Math.toRadians(270));
        robotElevator.setPosition(robotConstants.ELEVATOR_BOTTOM_POSITION);
    }
    private void doNothing()
    {

    }

    private void processStateMachine()
    {
        switch (currentAutonomousState)
        {
            case AUTON_START_STATE:
                processStateStart();
                break;
            case WAIT_PATH_DONE_STATE:
                processWaitPathDone();
                break;
            case MOVE_ELEVATOR_UP_STATE:
                processElevatorUpState();
                break;
            case WAIT_AUTO_FINISHED:
                processWaitAutonDoneState();
                break;
            case LOWER_ELEVATOR_SETUP_STATE:
                processLowerElevatorSetupState();
                break;
            case LOWER_ELEVATOR_STATE:
                processLowerElevatorState();
                break;
            case MOVE_TO_SAMPLE_FROM_OBSERVATION_ZONE_STATE:
                processMoveToSampleFromObservationZoneState();
                break;
            case MOVE_FROM_PICKUP_TO_SUBMERSIBLE_STATE:
                processMoveFromPickupToSubmersibleState();
            case DO_NOTHING:
                doNothing();



        }

    }


    public void loop() {
        processStateMachine();
        follower.update();
        telemetryA.addData("Test X: ", testX);
        telemetryA.addData("Test Y: ", testY);
        telemetryA.addData("Test Heading: ", testHeading);

        follower.telemetryDebug(telemetryA);
    }
}
