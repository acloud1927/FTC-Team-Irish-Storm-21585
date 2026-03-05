package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

//the counts and tolerances for odomentry is in the SimplifedOdometryRobot

@Autonomous(name="BackBlue", group = "Tempest")
public class BackBlue extends LinearOpMode
{

    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Back_right;
    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private Servo blocker;
    private boolean Shooting = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DcMotor axial;
    private DcMotor lateral;

    private ElapsedTime runtime = new ElapsedTime();
    static final double ENCODER_CLICKS = 537.7;    // REV 40:1  1120
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_CIRC = 3.78;
    int axialDist = 0;
    
    private int shootingTolerance = 100;
    
    private PIDController PID;


    @Override public void runOpMode()
    {

        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        lateral = hardwareMap.get(DcMotor.class, "leftLift");
        axial = hardwareMap.get(DcMotor.class, "rightLift");
        axialDist = axialDist + axial.getCurrentPosition();
        robot.initialize(true);

        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Get the webcam from the hardware map
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build the vision portal with the camera and AprilTag processor
        visionPortal = new VisionPortal.Builder().setCamera(webcamName).addProcessor(aprilTag).build();

        telemetry.addLine("Ready for start; initializing AprilTag detection...");
        telemetry.update();
        
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);


        if (true) {
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.addData("status","Axial,  %7d", axial.getCurrentPosition());
            telemetry.addData("status","0,  %7d", axialDist);
            telemetry.addData("status","Diff,  %7d", axialDist-axial.getCurrentPosition());
            telemetry.update();
        }

        PID = new PIDController(7, 0, 0.1);

        waitForStart();
        robot.resetHeading();

        if (opModeIsActive()) {
            
            new Thread(() -> {
                robot.drive(7, 1, 0);
                robot.turn(26.5, 0.8, 0);
            }).start();

            sleep(300);
            ShootingSequence(1300); //1570

            robot.turn(-90, 1, 0);
            robot.strafe(15.5, 0.5, 0);

            intake.setVelocity(1550);
            robot.drive(-44.5, 0.9, 0);
            robot.drive(38, 1, 0);
            intake.setVelocity(0);
            
            new Thread(() -> {
                sleep(800);
                intake.setPower(-1);
                sleep(40);
                intake.setPower(0);
                ShootingSequence(1300);  //1500
                robot.turn(22, 0.7, 0);
                robot.drive(37, 1, 0);
                robot.turn(-90, 1, 0);
            }).start();
            
            robot.turn(29, 0.7, 0);
            robot.drive(-13.5, 1, 0);
            sleep(5900);
            intake.setPower(1);
            robot.drive(-27, 0.9, 0);
            
            new Thread( () -> {
                sleep(3520);
                intake.setPower(-1);
                sleep(35);
                intake.setPower(0);
                ShootingSequence(1370);
            }).start();
            
            manualOverride(1400, -0.5, 0.9, -0.5, 0.9);
            robot.turn(90, 1, 0);
            manualOverride(2400, -1, 0.28, 0.28, -1);
            //robot.strafe(-30, 1, 0);
            robot.turn(27, 0.8, 0);
            sleep(1000000);
        }

    }

    public void manualOverride(int time, double FL, double FR, double BL, double BR) {
        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_right.setPower(FR);
        Front_left.setPower(FL);
        Back_right.setPower(BR);
        Back_left.setPower(BL);
        sleep(time);
        Front_right.setPower(0);
        Front_left.setPower(0);
        Back_right.setPower(0);
        Back_left.setPower(0);
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void ShootingSequence(int power) {
        blocker.setPosition(0.17);
        
        new Thread(() -> {
            for (int w = 0; w < 460; w++) {
                shooter.setVelocity(shooter.getVelocity() + PID.calculate(shooter.getVelocity(), power));
            }
            shooter.setVelocity(0);
        }).start();
        
        sleep(2000);
        blocker.setPosition(0);
        
        sleep(300);
        Intake(100);
        
        sleep(120);
        Intake(400);
        
        sleep(300);
        Intake(1500);
        
        blocker.setPosition(0.23);

    }
    
    private void Intake(int time) {
        intake.setVelocity(1500);
        sleep(time);
        intake.setVelocity(0);
    }

}


