package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import static org.firstinspires.ftc.teamcode.RobotConstants.STALL_T_DISREGARD_THRESHOLD;
import static org.firstinspires.ftc.teamcode.RobotConstants.STALL_X_DISREGARD_THRESHOLD;
import static org.firstinspires.ftc.teamcode.RobotConstants.STALL_Y_DISREGARD_THRESHOLD;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseRight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakePivot;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * Testing autonomous using Pedro based on curved back and forth example
 *
 */
@Config
@Autonomous
public class V3RightAuton1 extends OpMode {
    private RobotConstants robotConstants = null;
    private Telemetry telemetryA;
    private SubSystemGrabber robotGrabber;
    private SubSystemElevator robotElevator;
    private SubSystemRobotID robotRobotID = null;


    private Follower follower;
    private SubSystemIntakeArm robotIntakeArm = null;
    private SubSystemIntakeSlide robotIntakeSlide = null;
    private SubSystemIntakePivot robotIntakePivot = null;


    public double stallX = 0;
    public double previousStallX = 0;
    public double stallY = 0;
    public double previousStallY = 0;
    public double stallT = 0;
    public double previousStallT = 0;
    public double stallDeltaTotal;

    private int robotID;
    private int pickupCount = 0;
    public double testX = 0;
    public double testY = 0;
    public double test2X = 0;
    public double test2Y = 0;
    public double testHeading = 0;
    public int poseIndex;
    private enum AUTON_STATE
    {
        HOLD_POSE_STATE,
        TEST_STATE,
        AUTON_START_STATE,
        LOWER_ELEVATOR_STATE,
        LOWER_ELEVATOR_SETUP_STATE,
        WAIT_AUTO_FINISHED,
        WAIT_PATH_DONE_STATE,
        PUSH_SAMPLES_STATE,
        //MOVE_TO_SAMPLE_FROM_OBSERVATION_ZONE_STATE,
        PICKUP_SPECIMEN_STATE,
        WAIT_TIMER_DONE_STATE,
        ELEVATOR_PICK_UP_SAMPLE,
        BACKUP_AND_LOWER,
        MOVE_TO_SPECIMEN_PICKUP,
        DO_NOTHING,
        INITIALIZE_POSE_LIST_INDEX,
        FOLLOW_POSE_LIST
    }

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitPathDoneNextState = AUTON_STATE.DO_NOTHING;
    private AUTON_STATE lowerElevatorNextState = AUTON_STATE.DO_NOTHING;
    private AUTON_STATE waitTimerDoneNextState = AUTON_STATE.DO_NOTHING;
    private AUTON_STATE processFollowPoseListNextState = AUTON_STATE.DO_NOTHING;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;

    public static final Point startPoint = new Point (startingPoseRight.getX(), startingPoseRight.getY(), Point.CARTESIAN);
    public static final double wallY = startingPoseRight.getY() +0.8;
    public static final double specimenWallPickupX = 39;//47.5;
    public static final double specimenWallPickupY = wallY;//-61.5?
    public static final double pushPrepY = -12.8;
    public static final double pushAlignY = -12.8;
    public static final double pushObservationY = -43.5;//Was -44;
    public static final double behindSamples = -16.5;
    public static final double sample1X = 43;//was 43
    public static final double submersibleDropOffY = -31.8;
    public static final double sample2X = 53;
    public static final double sample3X = 51;

    public double listLastSegmentSpeed = 0.8;
    public static final Pose specimenZeroHangPose = pointAndHeadingToPose(-2.5, submersibleDropOffY, 90);
    public static Pose specimenOneHangPose = pointAndHeadingToPose(-1, submersibleDropOffY, 90);
    public static Pose specimenTwoHangPose = pointAndHeadingToPose(2, submersibleDropOffY, 90);
    public static Pose specimenThreeHangPose = pointAndHeadingToPose(5, submersibleDropOffY, 90);

    /*    public static final Pose testPose = pointAndHeadingToPose(40, -40, 90.0);*/



    //New Poses for submersible to spikes ready to push into human player area

    public static final Pose awayFromSubmersible = pointAndHeadingToPose(-2.5, -36.5, 90);//Moves back from submersible
    public static final Pose awayFromSubmersible2 = pointAndHeadingToPose(34.7, -36.5, 90);
    //public static final Pose submersibleToSpike1 = pointAndHeadingToPose(34.42,-49.88 , 90);//First point to move away from sub
    public static final Pose submersibleToSpike2 = pointAndHeadingToPose(33, behindSamples, 90);//Slide right ready to move behind samples
    public static final Pose submersibleToSpike3 = pointAndHeadingToPose(sample1X , behindSamples, 90);//Drive forward so behind samples
    public static final Pose submersibleToSpike3Push2Samples = pointAndHeadingToPose(51, behindSamples, 90);//NEW POINT
    public static final Pose submersibleToSpike4 = pointAndHeadingToPose(sample1X, pushObservationY , 90);//Push sample 1 to observation zone
    public static final Pose submersibleToSpike4Push2Samples = pointAndHeadingToPose(51, pushObservationY, 90);//NEW POINT
    public static final Pose submersibleToSpike5 = pointAndHeadingToPose(sample1X, behindSamples, 90);//Align for second sample
    public static final Pose submersibleToSpike5Push2Samples = pointAndHeadingToPose(55, behindSamples, 90);//NEW POINT
    public static final Pose submersibleToSpike6 = pointAndHeadingToPose(sample2X, behindSamples, 90);//Observation drop 'push to' location
    public static final Pose submersibleToSpike6Push2Samples = pointAndHeadingToPose(60, behindSamples, 90);//NEW POINT
    public static final Pose submersibleToSpike7 = pointAndHeadingToPose(sample2X, pushObservationY, 90);//Pushing second sample
    public static final Pose submersibleToSpike7Push2Samples = pointAndHeadingToPose(60, pushObservationY, 90);//NEW POINT
    //public static final Pose submersibleToSpike6 = pointAndHeadingToPose(56.67, behindSamples, 90); //Back behind the samples
    //public static final Pose submersibleToSpike8 = pointAndHeadingToPose(sample2X, behindSamples, 90);//Aligned behind second sample
    public static final Pose submersibleToSpike8 = pointAndHeadingToPose(40, pushObservationY + 5, 200);
    /*  public static final Pose submersibleToSpike9 = pointAndHeadingToPose(54,pushPrepY , 90);//Back behind samples
      public static final Pose submersibleToSpike10 = pointAndHeadingToPose(60,pushAlignY , 90);//Aligned behind third sample
      public static final Pose submersibleToSpike11 = pointAndHeadingToPose(60,pushObservationY  , 90);//Observation drop 'push to' location for third sample
      public static final Pose submersibleToSpike12 = pointAndHeadingToPose(52,-38  , 90);//Back out of Observation zone*/
    public static final Pose submersibleToSpike13 = pointAndHeadingToPose(specimenWallPickupX,specimenWallPickupY  , 270);//Move into grabbing position

    //public static final Pose testPoint1 = pointAndHeadingToPose(42.5, pushObservationY, 90);
    //public static final Pose testPoint2 = pointAndHeadingToPose(42.5, -pushObservationY, 90);
    public static final Pose[] getTwoSamples = {awayFromSubmersible, awayFromSubmersible2, submersibleToSpike2, submersibleToSpike3, submersibleToSpike4, submersibleToSpike5, submersibleToSpike6, submersibleToSpike7, submersibleToSpike8, submersibleToSpike13};
    //public static final Pose[] getDoubleSamplesPush = {submersibleToSpike1, submersibleToSpike2, submersibleToSpike3Push2Samples, submersibleToSpike4Push2Samples, submersibleToSpike5Push2Samples, submersibleToSpike6Push2Samples, submersibleToSpike7Push2Samples, submersibleToSpike8, submersibleToSpike13};
    //Paths
    //public static final Pose[] doTest = {testPoint1, testPoint2, testPoint1, testPoint2, testPoint1, testPoint2, testPoint1, testPoint2, testPoint1, testPoint2};
    /*
    public static final Path submersibleToSpikePathSegment1 = new Path(new BezierCurve(submersibleDropPoint, submersibleToSpike1, submersibleToSpike2));
    public static final Path submersibleToSpikePathSegment2 = new Path(new BezierCurve(submersibleToSpike2, submersibleToSpike3));
    public static final Path submersibleToSpikePathSegment3 = new Path(new BezierCurve(submersibleToSpike3, submersibleToSpike4));
    public static final Path submersibleToSpikePathSegment4 = new Path(new BezierCurve(submersibleToSpike4, submersibleToSpike5));
    public static final Path submersibleToSpikePathSegment5 = new Path(new BezierCurve(submersibleToSpike5, submersibleToSpike6));
    public static final Path submersibleToSpikePathSegment6 = new Path(new BezierCurve(submersibleToSpike6, submersibleToSpike7));
    public static final Path submersibleToSpikePathSegment7 = new Path(new BezierCurve(submersibleToSpike7, submersibleToSpike8));
    public static final Path submersibleToSpikePathSegment83x = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike9));
    public static final Path submersibleToSpikePathSegment82x = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike12));

    public static final Path submersibleToSpikePathSegment9 = new Path(new BezierCurve(submersibleToSpike9, submersibleToSpike10));
    public static final Path submersibleToSpikePathSegment10 = new Path(new BezierCurve(submersibleToSpike10, submersibleToSpike11));
    public static final Path submersibleToSpikePathSegment11 = new Path(new BezierCurve(submersibleToSpike11, submersibleToSpike12));
    public static final Path submersibleToSpikePathSegment12 = new Path(new BezierCurve(submersibleToSpike12, submersibleToSpike13));

     public static final PathChain submersibleToSpikeOneChain3x = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7, submersibleToSpikePathSegment83x, submersibleToSpikePathSegment9,submersibleToSpikePathSegment10, submersibleToSpikePathSegment11, submersibleToSpikePathSegment12);
    public static final PathChain submersibleToSpikeOneChain2x = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7, submersibleToSpikePathSegment82x, submersibleToSpikePathSegment12);
    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));

     */
    public static final Point submersibleDropPoint = new Point(2.4, -33, Point.CARTESIAN);
//    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));

    public static final double startToSubmersibleHeading = 90;
    public static final double submersibleToSpike1Heading = 90;

    //
    public static Pose specimenPickupPose = pointAndHeadingToPose(specimenWallPickupX,specimenWallPickupY  , 270);

    /*    public static final Path specimenTwoToSubmersible = new Path(new BezierCurve(specimenPickupPoint, specimenTwoClipOn));
        public static final Path moveToSpeciminThreePickup = new Path (new BezierCurve(specimenTwoClipOn, specimenPickupPoint));
        public static final Path specimenThreeToSubmersible = new Path(new BezierCurve(specimenPickupPoint, specimenThreeClipOn)); */
    public static Pose autonHoldLocation = pointAndHeadingToPose(35, -57, 0);

    /*   public static final Path moveToAutonHoldPath = new Path(new BezierCurve(specimenThreeClipOn, autonHoldLocation));*/


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void start() {
        restartTimeout(robotConstants.RIGHT_AUTON_DELAY);
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
        follower.setStartingPose(startingPoseRight);
        follower.update();

        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower.setMaxPower(robotConstants.START_TO_SUBMERSIBLE_SPEED);

        robotIntakeSlide.setPositionNow(robotConstants.INTAKE_SLIDE_START_POSITION);
        robotIntakeArm.setPositionNow(robotConstants.INTAKE_ARM_START_POSITION);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    private static Pose pointAndHeadingToPose(double x, double y, double headingInDegrees)
    {
        return new Pose(x, y, Math.toRadians(headingInDegrees));
    }
    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap, robotConstants.ELEVATOR_MULTIPLIER, robotConstants.ELEVATOR_MOTOR_COUNT);
        robotGrabber = new SubSystemGrabber(hardwareMap, 2);
        robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
        robotIntakeArm.setPositionNow(robotConstants.INTAKE_ARM_START_POSITION);
        robotIntakeSlide= new SubSystemIntakeSlide(hardwareMap);
        robotIntakeSlide.setPositionNow(robotConstants.INTAKE_SLIDE_START_POSITION);
        robotIntakePivot = new SubSystemIntakePivot(hardwareMap);
        robotIntakePivot.setPosition(robotConstants.INTAKE_PIVOT_START_POSITION);
    }

    /**
     * Setup a path to follow such that the bot ends facing in 'endHeading' direction
     */

    private void setupPath(Path pathToFollow, double endHeading)
    {

        double currentHeading = follower.getPose().getHeading();
        follower.followPath(pathToFollow, true);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, Math.toRadians(endHeading), 0.5);

    }
    private void setupPathChain(PathChain pathChainToFollow, double endHeading)
    {
        //MAKE SURE DEGREES OR RADIANS CORRECT

        follower.followPath(pathChainToFollow, true);
        //       int curveCount = pathChainToFollow.size();
        //       for (int i = 0; i < curveCount; i++)
        //       {
        //           pathChainToFollow.getPath(i).setLinearHeadingInterpolation(currentHeading, endHeading);
        //       }
    }
    void updateTelemetry()
    {
        telemetryA.addData("RobotID", robotID);
//        telemetryA.addData("Timer", timeoutTimer.time());
        telemetryA.addData("Current state",currentAutonomousState);
        telemetryA.addData("Elevator target: ", robotElevator.getTarget());
        telemetryA.addData("Elevator current: ", robotElevator.getPosition());
        telemetryA.addData("Left grabber position: ", robotGrabber.getLeftPosition());
        telemetryA.addData("StallX", stallX);
        telemetryA.addData("StallY", stallY);
        telemetryA.addData("StallT", stallT);
        telemetryA.addData("distanceMovedX", distanceMovedX);
        telemetryA.addData("distanceMovedY", distanceMovedY);
        telemetryA.addData("distanceMovedHeading", distanceMovedHeading);

        telemetryA.addData("stallDeltaTotal", stallDeltaTotal);

        telemetryA.addData("minDelta", minDelta);

        follower.telemetryDebug(telemetryA);
    }
    @Override
    public void init_loop()
    {
        robotID = robotRobotID.getRobotID();
        checkIfStalled();
        updateTelemetry();
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
        if (robotStalled)
            return false;
        else
            return follower.isBusy();
    }

    private boolean hasTimededout()
    {
        if (timeoutTimer.time() < timeoutPeriod)
            return false;
        else
            return true;
    }

    private void processStateStart()
    {
        if (hasTimededout())
        {
            robotIntakeSlide.setPosition(robotConstants.INTAKE_SLIDE_SAFE_POSITION);
            follower.setMaxPower(robotConstants.START_TO_SUBMERSIBLE_SPEED);
            setPathFromCurrentPositionToTargetPose(specimenZeroHangPose);
            restartTimeout(8000);
            robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
            currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
            waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_SETUP_STATE;
            lowerElevatorNextState = AUTON_STATE.INITIALIZE_POSE_LIST_INDEX;
            processFollowPoseListNextState = AUTON_STATE.PICKUP_SPECIMEN_STATE;
        }
    }

    private void processLowerElevatorSetupState()
    {
        robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_RELEASE);
        restartTimeout(500);
        currentAutonomousState = AUTON_STATE.LOWER_ELEVATOR_STATE;
    }
    private void processLowerElevatorState()
    {
//        if((Math.abs(robotElevator.getPosition() - robotConstants.ELEVATOR_TOP_RUNG_RELEASE) < 5) || hasTimededout())
        if(robotElevator.atTargetYet(10) || hasTimededout())
        {
            robotGrabber.setPosition(robotConstants.GRABBER_OPEN_POSITION);
            currentAutonomousState = lowerElevatorNextState;
            robotElevator.setPosition(robotConstants.ELEVATOR_SPECIMEN_PICKUP);
        }
    }

    private void processWaitPathDone()
    {
        if (!pathIsBusy() || robotStalled)
            currentAutonomousState = waitPathDoneNextState;
    }
    private void processWaitAutonDoneState()
    {
        follower.holdPoint(new BezierPoint(new Point(autonHoldLocation.getX(), autonHoldLocation.getY(), Point.CARTESIAN)), Math.toRadians(0));
        if (robotStalled)
        {
            follower.breakFollowing();
        }
        currentAutonomousState = AUTON_STATE.DO_NOTHING;
    }

    private void processHoldPoseState()
    {
        Point holdPoint = new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN);
        follower.holdPoint(new BezierPoint(holdPoint), follower.getPose().getHeading());
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.DO_NOTHING;
    }

    private void processMoveToSpecimenPickupState()
    {
        //setPathFromCurrentPositionToTargetPose(specimenPickupPose);
        setPathChainFromCurrentPositionToSpecimenPickup();
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.PICKUP_SPECIMEN_STATE;
    }
    private void processPickupSpecimenState()
    {
        pickupCount = pickupCount + 1;
        robotGrabber.setPosition(robotConstants.GRABBER_CLOSE_POSITION);
        restartTimeout(300);
        currentAutonomousState = AUTON_STATE.WAIT_TIMER_DONE_STATE;
        waitTimerDoneNextState = AUTON_STATE.ELEVATOR_PICK_UP_SAMPLE;
    }
    private void waitTimerDone()
    {
        if (hasTimededout())
        {
            currentAutonomousState = waitTimerDoneNextState;
        }
    }

    private void processElevatorPickUpSample()
    {
        robotElevator.setPosition(robotConstants.ELEVATOR_SPECIMEN_PICK_UP_LIFT_POSITION);
        if (robotElevator.atTargetYet(10))
        {
            restartTimeout(4000);
            //if (pickupCount == 3)// was 3, by changing it to 2, will stop after 2nd place
            //{
                //setPathFromCurrentPositionToTargetPose(specimenThreeHangPose);
                //lowerElevatorNextState = AUTON_STATE.BACKUP_AND_LOWER;
            //}
            if (pickupCount == 1) {

                setPathFromCurrentPositionToTargetPose(specimenOneHangPose);
                lowerElevatorNextState = AUTON_STATE.MOVE_TO_SPECIMEN_PICKUP;
            }
            else if (pickupCount == 2)
            {
                setPathFromCurrentPositionToTargetPose(specimenTwoHangPose);
                lowerElevatorNextState = AUTON_STATE.BACKUP_AND_LOWER;
            }
            robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
            currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
            waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_SETUP_STATE;
        }
    }
    private Point poseToPoint(Pose pose)
    {
        Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
        return returnPoint;
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
    private void setPathChainFromCurrentPositionToSpecimenPickup()
    {
        Pose test = new Pose (specimenWallPickupX, specimenWallPickupY, Math.toRadians(270));
        Point submersibleToPickupPointOne = new Point(13,-40, Point.CARTESIAN);
        Point submersibleToPickupPointTwo = new Point(32,-50, Point.CARTESIAN);
        Path segmentOnePath = new Path(new BezierCurve(poseToPoint(follower.getPose()), submersibleToPickupPointOne));
        segmentOnePath.setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(20), 1);

        Path segmentTwoPath = new Path(new BezierCurve(submersibleToPickupPointOne, submersibleToPickupPointTwo));
        segmentTwoPath.setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(270), 1);

        //Path segmentFinalPath = new Path(new BezierCurve(submersibleToPickupPointTwo, poseToPoint(specimenPickupPose)));
        Path segmentFinalPath = new Path(new BezierCurve(submersibleToPickupPointTwo, poseToPoint(test)));

        segmentFinalPath.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270), 1);
        PathChain pickupPath = new PathChain(segmentOnePath, segmentTwoPath, segmentFinalPath);
        follower.followPath(pickupPath, true);
    }

    /*private void setPathFromCurrentPositionToSubmersible(Pose endPose)
    {
        Point endPoint = poseToPoint(endPose);
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), specimenTwoHangPose)),90);
    }*/
    private void processBackupAndLowerState()
    {
        setPathFromCurrentPositionToTargetPose(autonHoldLocation);
        //follower.holdPoint(new BezierPoint(poseToPoint(autonHoldLocation)), Math.toRadians(180));
        robotElevator.setPosition(0);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
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
            case WAIT_AUTO_FINISHED:
                processWaitAutonDoneState();
                break;
            case LOWER_ELEVATOR_SETUP_STATE:
                processLowerElevatorSetupState();
                break;
            case LOWER_ELEVATOR_STATE:
                processLowerElevatorState();
                break;
            /*case PUSH_SAMPLES_STATE:
                processPushSampleToObservation();
                break;*/
            case PICKUP_SPECIMEN_STATE:
                processPickupSpecimenState();
                break;
            case WAIT_TIMER_DONE_STATE:
                waitTimerDone();
                break;
            case ELEVATOR_PICK_UP_SAMPLE:
                processElevatorPickUpSample();
                break;
            case MOVE_TO_SPECIMEN_PICKUP:
                processMoveToSpecimenPickupState();
                break;
            case BACKUP_AND_LOWER:
                processBackupAndLowerState();
                break;
            case DO_NOTHING:
                doNothing();
                break;
            /*case TEST_STATE:
                processTestState();
                break;*/
            /*case HOLD_POSE_STATE:
                processHoldPoseState();
                break;*/
            case INITIALIZE_POSE_LIST_INDEX:
                processInitializePoseListIndex();
                break;
            case FOLLOW_POSE_LIST:
                processFollowPoseList();
                break;
        }

    }


    private void processInitializePoseListIndex()
    {
        poseIndex = 0;
        setPathFromCurrentPositionToTargetPose(getTwoSamples[poseIndex]);//changed from getTwoSamples
        follower.setMaxPower(robotConstants.SUBMERSIBLE_TO_PUSH_SPEED);
        currentAutonomousState = AUTON_STATE.FOLLOW_POSE_LIST;
    }

    private void processFollowPoseList()
    {
        if (!follower.isBusy() || robotStalled)
        {
            poseIndex++;
            if (poseIndex < getTwoSamples.length)//changed from getTwoSamples
            {
                setPathFromCurrentPositionToTargetPose(getTwoSamples[poseIndex]);//changed from getTwoSamples
                if (poseIndex == getTwoSamples.length-1)
                {
                    follower.setMaxPower(listLastSegmentSpeed);
                }
                else
                {
                    follower.setMaxPower(robotConstants.SUBMERSIBLE_TO_PUSH_SPEED);
                }

            }
            else
            {
                currentAutonomousState = processFollowPoseListNextState;
                follower.setMaxPower(robotConstants.OBSERVATION_TO_SUBMERSIBLE_SPEED);

                //currentAutonomousState = AUTON_STATE.DO_NOTHING;

            }
        }
    }

    private double[] deltaTracking = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    private int stallIndex = 0;
    private double deltaTotal = 10;
    private double lastX = 0;
    private double lastHeading = 0;
    private double lastY = 0;
    private double minDelta = 1000;

    double distanceMovedX;
    double distanceMovedY;
    double distanceMovedHeading;

    public boolean robotStalled = false;

    public void checkIfStalled()
    {
        double distanceMoved = 0;
        if (follower.isBusy()) {
            double currentX = follower.getPose().getX();
//            if (Math.abs(currentX - previousStallX) > STALL_X_DISREGARD_THRESHOLD)
//            {
//                currentX  = previousStallX;
//            }
//            if (Math.abs(currentY - previousStallY) > STALL_Y_DISREGARD_THRESHOLD)
//            {
//                currentX  = previousStallX;
//            }
//            if (Math.abs(currentX - previousStallX) > STALL_T_DISREGARD_THRESHOLD)
//            {
//                currentX  = previousStallX;
//            }

            distanceMovedX = Math.abs(currentX - lastX);

            if (distanceMovedX > STALL_X_DISREGARD_THRESHOLD)
            {
                //currentX = lastX;
            }

            lastX = currentX;
            double currentY = follower.getPose().getY();
            distanceMovedY = Math.abs(currentY - lastY);
            if (distanceMovedY > STALL_Y_DISREGARD_THRESHOLD)
            {
                //currentY = lastY;
            }
            lastY = currentY;
            double currentHeading = follower.getPose().getHeading();
            distanceMovedHeading = Math.abs(currentHeading - lastHeading);
            if (distanceMovedHeading > 2*Math.PI)
            {
                distanceMovedHeading = Math.abs(distanceMovedHeading - (2*Math.PI));
            }
            distanceMovedHeading = distanceMovedHeading*50;
            if (distanceMovedHeading > STALL_T_DISREGARD_THRESHOLD)
            {
                distanceMovedHeading = 0.2;
                //currentHeading = lastHeading;
            }


            lastHeading = currentHeading;
            stallX = distanceMovedX;
            stallY = distanceMovedY;
            stallT = distanceMovedHeading;
            distanceMoved = stallX + stallY + stallT;
            stallDeltaTotal = deltaTotal;
            deltaTotal = deltaTotal - deltaTracking[stallIndex] + distanceMoved;

            if (deltaTotal < minDelta)
                minDelta = deltaTotal;
            deltaTracking[stallIndex] = distanceMoved;
            if (stallIndex < 9)
                stallIndex = stallIndex + 1;
            else
                stallIndex = 0;

            if (deltaTotal < robotConstants.STALL_THRESHOLD)
                robotStalled = true;
            else
                robotStalled = false;

        }
        else
        {
            lastX = follower.getPose().getX();
            lastY = follower.getPose().getY();
            lastHeading = follower.getPose().getHeading();



        }
    }


    public void loop() {
        processStateMachine();
        follower.update();

        checkIfStalled();

        robotIntakeSlide.updateServoPosition();
        robotIntakeArm.updateArmPosition();
        updateTelemetry();
    }
}