package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "TempestTeleopPID")
public class Teleop extends LinearOpMode {

    private IMU imu;

    private DcMotor Back_right;
    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    
    private DcMotorEx shooter;
    private DcMotorEx intake;
    
    private DcMotor leftLift;
    private DcMotor rightLift;
    private Servo blocker;
    
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    private double waitingTime = 50;
    private double targetSpeed = 1400;
    private boolean shootinGooooood = false;
    private double shootingTolerance = 60;
    private boolean Shooting = false;
    private boolean REY1 = false;
    private boolean previousA = false;
    
    private double turningPower = 0;
    
    ElapsedTime tick = new ElapsedTime();
    boolean waitingToShoot = false;

    private double angle;
    private double direction;
    private double dx;
    private double dy;
    
    private PIDController PID;
  

    @Override
    public void runOpMode() {
        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        blocker = hardwareMap.get(Servo.class, "blocker");
        imu = hardwareMap.get(IMU.class, "imu");

        
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Get the webcam from the hardware map
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build the vision portal with the camera and AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTag)
                .build();
                
        // Wait for the camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
            sleep(20);
        }
        
        setManualExposure(8);  // exposure ms


        
        telemetry.addLine("Ready for start; initializing AprilTag detection...");
        telemetry.update();

        shooter.setVelocity(0);
        imu.resetYaw();
        
        PID = new PIDController(7, 0, 0.1);
        
        waitForStart();

     
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back_left.setDirection(DcMotor.Direction.REVERSE);
        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_left.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
     
        if (opModeIsActive()) {
            try {
            while (opModeIsActive()) {
                DriveTrain();
                Telemetry();
                Shooter();
                Intake();
                Lift();
                Block();
                IntakeAdjustment();
            }
            } finally {
                
            }
        }
    }
    
    private void setManualExposure(int exposureMS) {
        if (visionPortal == null) return;
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl == null) return;
        // Turn off auto exposure
        exposureControl.setMode(ExposureControl.Mode.Manual);
        // Set exposure time
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
    }
    
    private void IntakeAdjustment() {
        if (gamepad2.y && REY1 == false) {
            Shooting = true;
            intake.setVelocity(-1500);
            sleep(70);
            intake.setVelocity(0);
            Shooting = false;
            REY1 = true;
        } else if (gamepad2.y) {
            REY1 = true;
        } else {
            REY1 = false;
        }
    }

    private void Block() {
        if (gamepad2.b == true) {
            blocker.setPosition(0);
        } else if (gamepad2.a == true) {
            blocker.setPosition(0.2);
        }
    }
    
    private boolean withinShootingRange() {
        return (shooter.getVelocity() > targetSpeed - shootingTolerance) && (shooter.getVelocity() < targetSpeed + shootingTolerance);
    }

    private void Shooter() {
        if (gamepad2.dpad_up) {
            waitingTime += 2;
        } else if (gamepad2.dpad_down) {
            waitingTime -= 2;
        }
        if (gamepad2.left_trigger > 0) {

            
            //shooter.setVelocity((2 - (shooter.getVelocity()/targetSpeed)) * targetSpeed);
            //shooter.setVelocity(targetSpeed);
            shooter.setVelocity(shooter.getVelocity() + PID.calculate(shooter.getVelocity(), targetSpeed));
            
            if (withinShootingRange()) {
                if (!waitingToShoot) {
                    tick.reset();
                    waitingToShoot = true;
                    blocker.setPosition(0);
                }
            
                if (tick.milliseconds() >= waitingTime) {//(2400 - (20*shootingTolerance))) {
                    if (withinShootingRange()) {
                        shootinGooooood = true;
                    } else {
                        shootinGooooood = false;
                    }
                    waitingToShoot = false;
                }
            } else {
                waitingToShoot = false;
                shootinGooooood = false;
            }
        } else if (gamepad2.left_bumper) {
            shooter.setVelocity(-1300);
        } else {
            shooter.setVelocity(0);
            blocker.setPosition(0.2);
        }
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        if (currentDetections.size() == 1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.ftcPose != null) {
                    //targetSpeed = Math.min(Math.max(6.7 * detection.ftcPose.y + 922, 1380), 1650);
                    targetSpeed = Math.min(4.94 * detection.ftcPose.y + 836.28, 1330);
                    waitingTime = 0.85 * detection.ftcPose.y + 10; //- 03.09;
                    shootingTolerance = (-(0.2) * detection.ftcPose.y) + 50;
                }
            }
        } 
        
    }

    private void Intake() {
        if (gamepad2.right_trigger > 0 || shootinGooooood == true) {
            intake.setVelocity(2000);
        } else if (gamepad2.right_bumper) {
            intake.setVelocity(-2000);
        } else if (Shooting == false) {
            intake.setVelocity(0);
        }
    }

    private void Lift() {
        
        if (gamepad1.x) {
            leftLift.setPower(0.95);
            rightLift.setPower(1);
        } else if (gamepad1.b) {
            leftLift.setPower(-0.95);
            rightLift.setPower(-1);
        } else {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
    }
    
    private void Orient() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        direction = -gamepad1.right_stick_x;
        
        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        dx = x * Math.cos(-heading) - y * Math.sin(-heading);
        dy = x * Math.sin(-heading) + y * Math.cos(-heading);
                
        if (gamepad1.a && !previousA) {
            imu.resetYaw();
        }
        previousA = gamepad1.a;
        
    }

    private void DriveTrain() {
        Orient();
        if (!gamepad1.dpad_left) {
            double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, direction + dy + dx)); 
            Back_left.setPower((dy + (dx - direction)) * denominator); 
            Front_left.setPower((dy - (dx + direction)) * denominator); 
            Back_right.setPower((dy - (dx - direction)) * denominator); 
            Front_right.setPower((dy + dx + direction) * denominator);
                    
        } else {
            
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
            if (currentDetections.size() == 1) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 20 && detection.ftcPose != null) {
                        
                        turningPower = (detection.ftcPose.z - 7) * 0.02;
                        
                        Back_left.setPower(turningPower);
                        Front_left.setPower(turningPower);
                        Back_right.setPower(-turningPower);
                        Front_right.setPower(-turningPower);
                        
                    } else if (detection.id == 24 && detection.ftcPose != null) {
                        
                        turningPower = (detection.ftcPose.z - 1) * 0.02;
                        
                        Back_left.setPower(turningPower);
                        Front_left.setPower(turningPower);
                        Back_right.setPower(-turningPower);
                        Front_right.setPower(-turningPower);
                        
                    } 
                }
            } 
        }
    }
    
    private void Telemetry() {
        telemetry.addData("BACK_LEFT", Back_left.getPower());
        telemetry.addData("BACK_RIGHT", Back_right.getPower());
        telemetry.addData("FRONT_RIGHT", Front_right.getPower());
        telemetry.addData("FRONT_LEFT", Front_left.getPower());
        telemetry.addData("Shooting Power", shooter.getVelocity());
        telemetry.addData("Target Power", targetSpeed);
        telemetry.addData("Waiting Time", waitingTime);
        telemetry.addData("PID output", PID.calculate(shooter.getVelocity(), targetSpeed));

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("# Detected", currentDetections.size());
    
            if (currentDetections.size() == 1) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.ftcPose != null) {
                        telemetry.addLine(String.format("ID: %d", detection.id));
                        telemetry.addLine(String.format("Pose: X=%.1f Y=%.1f Z=%.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        if (shootinGooooood) {
                            telemetry.addData("", "\n" +
                                "------------------------\n" +
                                "|                   /       |\n" +
                                "|                 /         |\n" +
                                "|    l          /           |\n" +
                                "|       l     /             |\n" +
                                "|          v                |\n" +
                                "------------------------\n");
                        } else {
                            telemetry.addData("", "\n" +
                                "------------------------\n" +
                                "|                           |\n" +
                                "|                           |\n" +
                                "|                           |\n" +
                                "|                           |\n" +
                                "|                           |\n" +
                                "------------------------\n");
                        }
                    }
                }
            } else {
                telemetry.addData(""," ");
            }
     
        
      
        telemetry.update();
        
    }

}
