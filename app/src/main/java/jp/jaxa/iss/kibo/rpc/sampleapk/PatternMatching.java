package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class PatternMatching extends KiboRpcService {
    @SuppressLint("DefaultLocale")
    @Override
    protected void runPlan1() {
        // The mission starts.
        api.startMission();

        // Move to a point.

        Mat cropped = moveToObject(10.9d, -9.92284d, 5.195d, 0f, 0f, -0.707f, 0.707f, -0.18, -0.15, 0);
        Mat gray = new Mat();
        if (cropped.type() != CvType.CV_8UC3) {
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_GRAY2BGR);
        }

        // Convert the source image to 8-bit 3-channel
        Mat img = new Mat();
        cropped.convertTo(img, CvType.CV_8UC3);
        Imgproc.cvtColor(cropped, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply Canny
        Mat TarImg = new Mat();
        Imgproc.Canny(gray, TarImg, 0, 255);
        api.saveMatImage(TarImg, "edges");

        patternMatching(TarImg);
//        Mat segment = new Mat();


        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */



        // Pattern matching
        // Load template images



        // turn on the front flash light
//        api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = yourMethod();

        // turn off the front flash light
//        api.flashlightControlFront(0.00f);



        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

//        Point point2 = new Point(11.05d, -9.00d, 5.13d);
//        Quaternion quaternion2 = new Quaternion(0f, 0.707f, 0f, 0.707f);
//        api.moveTo(point2, quaternion2, true);

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
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
    public String yourMethod() {
        return "your method";
    }

    // Resize image
    private Mat resizeImg(Mat templ, int width) {
        int heigth = (int) (templ.rows() * ((double) width / templ.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(templ, resizedImg, new Size(width, heigth), 0, 0, Imgproc.INTER_LANCZOS4);

        return resizedImg;
    }

    // Rotate image
    private Mat rotImg(Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());

        return rotatedImg;
    }

    // Remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double length = 15; // Width 10 px
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);
                if (distance <= length) {
                    isInclude = true;
                    break;
                }
            }

            if (!isInclude) {
                filteredList.add(point);
            }
        }
        return filteredList;
    }

    // Find the distance between two points
    private static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    }

    // Get the maximum value of an array
    private int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;

        // Find the index of the element with the largest value
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    private Mat moveToObject(double xCord, double yCord, double zCord, float qx, float qy, float qz, float qw, double dispX, double dispY, double dispZ)
    {
        Mat cropped = new Mat();
        Point point = new Point(xCord, yCord, zCord);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);
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
            if (newX < 10.9){
                dispX = -dispX;
            }

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

            Aruco.estimatePoseSingleMarkers(corners_2, markerLength, cameraMatrix, distCoeffs_2, rvecs_2, tvecs_2);
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
            Calib3d.undistort(rotatedImage, undistortImg, cameraMatrix_2, cameraCoefficients);

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
            cropped = new Mat(undistortImg, roi);
            api.saveMatImage(cropped, "cropped image");
            System.out.println("Image cropped and saved successfully");
        }
        return cropped;
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

    public void patternMatching(Mat cropped){

        org.opencv.core.Point matchLoc = new org.opencv.core.Point();
        Core.MinMaxLocResult mmlr = new Core.MinMaxLocResult();
        String[] TEMPLATE_FILE_NAME = {"beaker.png", "goggle.png", "hammer.png", "kapton_tape.png", "pipette.png", "screwdriver.png", "thermometer.png", "top.png", "watch.png", "wrench.png"};
        String[] TEMPLATE_NAME = {"beaker", "goggle", "hammer", "kapton_tape", "pipette", "screwdriver", "thermometer", "top", "watch", "wrench"};
        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i < TEMPLATE_FILE_NAME.length; i++) {
            try {
                // Open the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                //Convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                Mat gray = new Mat();
                if (mat.type() != CvType.CV_8UC3) {
                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
                }

                // Convert the source image to 8-bit 3-channel
                Mat img = new Mat();
                mat.convertTo(img, CvType.CV_8UC3);
                Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);

                // Apply Canny

                // Assign to an  array  of templates
                templates[i] = gray;

                inputStream.close();

            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // Number of  matches for each template
        int templateMatchCnt[] = new int[10];

        // Get the number of template matches
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            // Number of matches
            int matchCnt = 0;

            // Coordinates of the matched locatioin
            List<org.opencv.core.Point> matches = new ArrayList<>();

            // Loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = cropped.clone();

            // Pattern matching
            int widthMin = 55; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 1; //[px]
            int changeAngle = 45; //[degree]

//            for (double scale : scales){
            for (int i = widthMin; i <= widthMax; i += changeWidth) {
                for (int j = 0; j <= 360; j += changeAngle) {
//                    Mat resizedTemp = new Mat();
//                    Imgproc.resize(template, resizedTemp, new Size(template.cols() * scale, template.rows() * scale));
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);
                    Mat blurredImage = new Mat();
                    Imgproc.GaussianBlur(rotResizedTemp, blurredImage, new Size(5, 5), 0);
                    api.saveMatImage(blurredImage, "blurredImage");
                    Mat edges = new Mat();
                    Imgproc.Canny(blurredImage, edges, 0, 255);
                    api.saveMatImage(edges, "edges");

//                    api.saveMatImage(rotResizedTemp, "no");
                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, edges, result, Imgproc.TM_CCOEFF_NORMED);
                    // Get  coordinates with similarity grater than or equal to the threshold
                    double threshold = 0.4;
                    mmlr = Core.minMaxLoc(result);

                    double maxVal = mmlr.maxVal;
                    System.out.println("Size: " + i + " Angle: " + j + " maxVal: " + maxVal);
                    if (maxVal >= threshold) {
                        // Extract only results grater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        matchLoc = mmlr.maxLoc;
                        api.saveMatImage(rotResizedTemp, String.format("%s", TEMPLATE_NAME[tempNum]));

                        // Draw rectangle around the detected object
                        Imgproc.rectangle(
                                cropped,
                                matchLoc,
                                new org.opencv.core.Point(matchLoc.x + rotResizedTemp.cols(), matchLoc.y + rotResizedTemp.rows()),
                                new Scalar(255, 0, 0),
                                1
                        );

                        api.saveMatImage(cropped, String.format("Final %s %d", TEMPLATE_NAME[tempNum], templateMatchCnt[tempNum]));

                        // Get match counts
                        for (int y = 0; y < thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
//                    else {
//                        System.out.println("Template/Image not detected");
//                    }
                }
            }

            // Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt += filteredMatches.size();

            // Number of matches for each template
            templateMatchCnt[tempNum] = matchCnt;
            System.out.println("Match Count of: " + TEMPLATE_NAME[tempNum] + " is " + matchCnt);
        }
        // When you recognize items, let’s set the type and number.
        int mostMatchTemlatenNum = getMaxIndex(templateMatchCnt);
        Mat img = templates[mostMatchTemlatenNum].clone();
        // Locate the match location


        System.out.println("name : " + TEMPLATE_NAME[mostMatchTemlatenNum] + " number of images : " + templateMatchCnt[mostMatchTemlatenNum]);
        api.setAreaInfo(1, TEMPLATE_NAME[mostMatchTemlatenNum], templateMatchCnt[mostMatchTemlatenNum]);
    }
}