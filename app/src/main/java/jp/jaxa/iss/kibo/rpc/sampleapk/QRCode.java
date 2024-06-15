package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class QRCode extends KiboRpcService {
    @Override
    protected void runPlan1() {
        // The mission starts.
        api.startMission();

        // Move to a point.
        moveToObject(10.9d, -9.92284d, 5.195d, 0f, 0f, -0.707f, 0.707f, -0.18, -0.15, 0);


        // When you recognize items, let’s set the type and number.
        api.setAreaInfo(1, "item_name", 1);

        // Move to the each area and recognize the items.

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        // Recognize which item the astronaut has.
        api.notifyRecognitionItem();

        // Move Astrobee to the location of the target item.
        api.takeTargetItemSnapshot();

    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here.
    }

    // You can add your method.
    private void moveToObject(double xCord, double yCord, double zCord, float qx, float qy, float qz, float qw, double dispX, double dispY, double dispZ)
    {
        Point point = new Point(xCord, yCord, zCord);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        Mat init_location = api.getMatNavCam();

        // Detect AR
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(init_location, dictionary, corners, markerIds);
        float markerLength = 0.05f;

        // Check if any markers are detected
        if (!corners.isEmpty()) {
            // Code for Moving close to Object
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
            Mat distCoeffs = new MatOfDouble(api.getNavCamIntrinsics()[1]);

            Mat rvecs = new Mat();
            Mat tvecs = new Mat();

            Aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            Mat rvec = rvecs.row(0);
            Mat tvec = tvecs.row(0);

            double[] tvecArray = new double[3];
            tvec.get(0, 0, tvecArray);
            double arucoX = tvecArray[0];
            double arucoY = tvecArray[1];
            double arucoZ = tvecArray[2];

            System.out.println("Marker ID: " + markerIds);
            System.out.println("Translation Vector: " + tvecs.dump());
            System.out.println("Rotation Vector: " + rvecs.dump());

            System.out.println("Aruco X: " + arucoX);
            System.out.println("Aruco Y: " + arucoY);
            System.out.println("Aruco Z: " + arucoZ);

            double newX = xCord + arucoZ;
            double newY = yCord + arucoX;
            double newZ = zCord + arucoY;
            Point correctCords = new Point(newX + dispX, newY + dispY, newZ + dispZ);
            Quaternion correctQuarternion = new Quaternion(qx, qy, qz, qw);
            api.moveTo(correctCords, correctQuarternion, true);

            Mat final_location = api.getMatNavCam();
            api.saveMatImage(final_location, "final location");

            // Code for rotating the image captured
            Dictionary dictionary_2 = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners_2 = new ArrayList<>();
            Mat markerIds_2 = new Mat();
            Aruco.detectMarkers(final_location, dictionary_2, corners_2, markerIds_2);
            Mat cameraMatrix_2 = new Mat(3, 3, CvType.CV_64F);
            cameraMatrix_2.put(0, 0, api.getNavCamIntrinsics()[0]);
            Mat distCoeffs_2 = new MatOfDouble(api.getNavCamIntrinsics()[1]);

            Mat rvecs_2 = new Mat();
            Mat tvecs_2 = new Mat();

            Aruco.estimatePoseSingleMarkers(corners_2, markerLength, cameraMatrix_2, distCoeffs_2, rvecs_2, tvecs_2);
            Mat rvec_2 = rvecs_2.row(0);
            Mat tvec_2 = tvecs_2.row(0);

            double rotationAngle = calculateRotationAngle(rvecs_2);
            Mat rotatedImage = rotateImage(final_location, rotationAngle);
            api.saveMatImage(rotatedImage, "rotated image");

            // Undistort Image
            Mat cameraCoefficients = new Mat( 1,  5, CvType.CV_64F);
            cameraCoefficients.put(  0,  0, api.getNavCamIntrinsics()[1]);
            cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

            Mat undistortImg = new Mat();
            Calib3d.undistort(rotatedImage, undistortImg, cameraMatrix, cameraCoefficients);

            // Code for cropping the rotated image
            Dictionary dictionary_3 = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners_3 = new ArrayList<>();
            Mat markerIds_3 = new Mat();
            Aruco.detectMarkers(undistortImg, dictionary_3, corners_3, markerIds_3);

            Mat cornersMat = corners_3.get(0);

            System.out.println("Marker ID: " + markerIds_3.get(0, 0)[0]);
            double[] corner = cornersMat.get(0, 3);
            int corner_x = (int) corner[0];
            int corner_y = (int) corner[1];
            System.out.println("Corner: (" + corner_x + ", " + corner_y + ")");

            if (final_location.empty()) {
                System.out.println("Could not open or find the image");
            }
            Rect roi = new Rect(corner_x-195, corner_y-110, 280, 200);

            // Crop the image using the defined ROI
            Mat cropped = new Mat(undistortImg, roi);
            api.saveMatImage(cropped, "cropped image");

            System.out.println("Image cropped and saved successfully");
        }
    }

    private static double calculateRotationAngle(Mat rvecs) {
        // Extract the rotation vector for the first detected marker
        Mat rvec = rvecs.row(0);

        // Convert the rotation vector to a rotation matrix
        Mat rotationMatrix = new Mat();
        Calib3d.Rodrigues(rvec, rotationMatrix);

        // Extract the angle of rotation around the Z-axis from the rotation matrix
        // The rotation matrix is a 3x3 matrix, we need to get the angle from it
        double angle = Math.atan2(rotationMatrix.get(1, 0)[0], rotationMatrix.get(0, 0)[0]);

        // Convert the angle to degrees
        return Math.toDegrees(angle);
    }

    private static Mat rotateImage(Mat image, double angle) {
        // Rotate the image by specified angle
        Mat rotatedImage = new Mat();
        int image_col = image.cols()/2;
        int image_row = image.rows()/2;
        org.opencv.core.Point center = new org.opencv.core.Point(image_col, image_row);
        Mat rotationMatrix = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Imgproc.warpAffine(image, rotatedImage, rotationMatrix, image.size());
        return rotatedImage;
    }
}