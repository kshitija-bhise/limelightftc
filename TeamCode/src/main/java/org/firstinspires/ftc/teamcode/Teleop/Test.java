//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
//import org.firstinspires.ftc.teamcode.Hardware.Drive;
//import org.firstinspires.ftc.teamcode.Hardware.Shooter;
//
//@TeleOp(name = "TestV1")
//public class Test extends LinearOpMode {
//    DcMotor LR, RR, LF, FR;
////    Shooter shooter;
////    ActiveIntake Intake;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        LR = hardwareMap.dcMotor.get("LR");
//        RR = hardwareMap.dcMotor.get("RR");
//        LF = hardwareMap.dcMotor.get("LF");
//        FR = hardwareMap.dcMotor.get("RF");
//
//        Shooter shooter = new Shooter(hardwareMap);
//
//        LR.setDirection(DcMotorSimple.Direction.REVERSE);
//        LF.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Drive drive = new Drive(LF, FR, LR, RR);
//        ActiveIntake intake = new ActiveIntake(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            drive.drive(gamepad1);
//
//            if(gamepad1.dpad_up){
//                shooter.startShooter();
//            }
//            else{
//                shooter.stopShooter();
//            }
//
//            if(gamepad1.a){
//                intake.startIntake();
//            }
//            else {
//                intake.stopIntake();
//            }
//
//            if (gamepad1.x){
//                shooter.shoot();
//                sleep(1000);
//                shooter.resetServo();
//            }
//        }
//    }
//}
