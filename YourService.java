package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

//import java.util.Dictionary;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.io.IOException;
import java.io.InputStreamReader;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */



public class YourService extends KiboRpcService {

    private static final String[] TEMPLATE_FILE_NAME = {
            "beker.png",
            "goggle.png",
            "hammer.png",
            "kapton_tape.png",
            "pipette.png",
            "screwdriver.png",
            "thermometer.png",
            "top.png",
            "watch.png",
            "wrench.png",
            // ... add more file names for other item templates
    };


    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
//        Log.i(TAG, msg:"start mission");

        // The mission starts.
        api.startMission();

        // Move to a point.
        Point point = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);

        Result results = api.moveTo(point, quaternion, false);


        // Get a camera image.
        Mat image = api.getMatNavCam();

        /* *********************************************************************** */
        /* Write your code to recognize type and number of items in the each area! */
        /* *********************************************************************** */

        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        //Get camera Matrix
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        //Get len distortion parameters
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);
        //Undistort image
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        //pattern matching
        //local template images


        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i < TEMPLATE_FILE_NAME.length; i++) {
            try {
                //open the template image in Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                //convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                //Assign to an array of templates
                templates[i] = mat;
                inputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        //Number of matches for each template
        int templateMatchCnt[] = new int[10];

        //Get the number of template matches
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            //Number of mathes
            int matchCnt = 0;

            //coordinates of the method location
            List<org.opencv.core.Point> matches = new ArrayList<>();

            //loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();

            //pattern matching
            int widthMin = 20; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 5; //[px]
            int changeAngle = 45; //[degree]

            for (int i = widthMin; i <= widthMax; i += changeWidth) {
                for (int j = 0; j <= 360; j += changeAngle) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    //Get coordinates with similarity greater than or equal to the threshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold) {
                        //Extract only result greater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        //Get match counts
                        for (int y = 0; y < thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));

                                }
                            }
                        }
                    }

                }
            }
        }

        //Avoid detecting the same location multiple times
        List<org.opencv.core.Point> matches = new ArrayList<>(); // Move declaration outside the loop
        int matchCnt = 0; // Move declaration outside the loop

        //Get the number of template matches
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            //Reset match count for each template
            matchCnt = 0;
            matches.clear(); // Clear the list for each template

            //loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();

            //pattern matching
            int widthMin = 20; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 5; //[px]
            int changeAngle = 45; //[degree]

            for (int i = widthMin; i <= widthMax; i += changeWidth) {
                for (int j = 0; j <= 360; j += changeAngle) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    //Get coordinates with similarity greater than or equal to the threshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold) {
                        //Extract only result greater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        //Get match counts
                        for (int y = 0; y < thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
                }
            }

            //Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt += filteredMatches.size();

            //Number of matches for each template
            templateMatchCnt[tempNum] = matchCnt;
        }

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
    private String yourMethod() {

        return "your method";
    }


    //Resize image
    private Mat resizeImg(Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, height));

        return resizedImg;
    }

    //Rotate Image
    private Mat rotImg(Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());

        return rotatedImg;
    }

    //remove multiple detections
    private static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double length = 10;
        List<org.opencv.core.Point> filteredList = new ArrayList<>();
        for (org.opencv.core.Point point : points) {
            boolean isIncluede = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);

                if (distance <= length) {
                    isIncluede = true;
                    break;
                }
            }

            if (!isIncluede) {
                filteredList.add(point);
            }
        }

        return filteredList;
    }

    //find the distance between two points
    private static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    //Get the maximum value of an array
    public int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;

        //find the index of the element with the largest value
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;

            }
        }
        return maxIndex;
    }
}


// video: 6:05


