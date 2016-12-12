#include "includes.h"
#include <unistd.h>


int main( int argc, char** argv )
{
    Parser p(argc, argv, keys);
    p.about("Application name v1.0.0\n");
    if(p.has("help")){
        p.printMessage();
        return 0;
    }

    if(p.has("calib"))
    {
	chdir("../cam_calib/");
        system("../cam_calib/camera_calibration");
        return 0;
    }

    bool simulation = p.has("simulation");

    String camera_file = p.get<String>("cp");
    String patterns = p.get<String>("p");

    Mat grayImage, binaryImage, frame;

    Mat cameraMatrix;
    Mat distCoeffs;

    std::vector<Mat> trainingVec, negativesVec, gradiend, patternLibrary;
    std::vector<int> labels, ids;
    std::vector<Pattern> detectedPattern;
    std::vector<Point> locations, approx;
    std::vector<std::vector<Point>>contours;
    std::vector<Vec4i> hierarchy;
    std::vector<Point2f> vertices;
    std::vector<std::vector<Point2f>> cor;

    VideoCapture cap(0);
    loadImages(patterns, patternLibrary);


    int norm_pattern_size = 80;
    int frame_size = 640;
    int vertex = -1;
    int adapt_block_size = 45;
    double adapt_thresh = 5;
    double confidenceThreshold = 0.75;

    if(!cap.isOpened() || patternLibrary.size() == 0)
        return -1;

    ArucoDetector ad;
    ContourFinder cf(adapt_thresh, adapt_block_size);
    RoiDetector myDetector(adapt_thresh, adapt_block_size, norm_pattern_size);

    TamplateMatcher  tm(patternLibrary, confidenceThreshold,norm_pattern_size);

    std::unique_ptr<Robot> r;
    if (simulation) r = std::make_unique<Robot>(MAX_VEL, MAX_ANG_VEL, ZRYCHLENI, 0.02/2, 0.7/2);
    else r = std::make_unique<Robot>(MAX_VEL, MAX_ANG_VEL, ZRYCHLENI, 0.02/2, 0.7/2, IP, port);

    PID distanceController(DIST_KP, DIST_KI, DIST_KD, 0.00, MAX_VEL, -MAX_VEL,  5);
    PID angleController(ANGLE_KP, ANGLE_KI, ANGLE_KD, 1, MAX_ANG_VEL, -MAX_ANG_VEL, 0.15);

    CCameraCalibration calibrator;

    calibrator.readFromFile(camera_file);

    cameraMatrix = calibrator.cameraMatrix;

    distCoeffs = calibrator.distortionCoefficients;
    double fovx = 2*atan((2*cameraMatrix.at<double>(0))/frame_size);
    Measurement m(frame_size, 0.08, 30, 155, fovx);

    cv::Point2f q(255,100);
    clock_t start, stop;
    double sample;
    double q_x = 0.25;
    double q_y = 0.5;
    double angle = CV_PI;
    double dist = 100;

    while(true)
    {
        start = clock();

        Mat imcopy;
        cap >> frame;
        frame.copyTo(imcopy);
        detectedPattern.clear();
        cf.binarize(frame, grayImage, binaryImage);
        cf.contours(binaryImage, contours, hierarchy);

        for(int i = 0; i < contours.size(); i++)
        {
            locations.clear();
            vertices.clear();
            myDetector.detectROI(contours[i], hierarchy, i, grayImage, vertices, vertex);

            if(vertices.size() == 4)
            {
                tm.detect(myDetector.getNormROI(), cameraMatrix, distCoeffs, vertices, vertex, detectedPattern);
            }
        }

        for(int i = 0; i < detectedPattern.size(); i++)
        {
            detectedPattern[i].draw(frame, cameraMatrix, distCoeffs);
        }


        vector< Vec3d > rvecs, tvecs;

        ad.detect(imcopy, cor, ids);
        if(ids.size() > 0)
            ad.drawDetected(imcopy, cor, ids);


        if(cor.size() > 0)
        {
            cv::aruco::estimatePoseSingleMarkers(cor,0.08, cameraMatrix, distCoeffs, rvecs, tvecs);
            angle = m.angle(cor.at(0));
            dist = m.distance(cor.at(0));
        }

        stop = clock();
        sample = (double)(stop - start) / CLOCKS_PER_SEC;

        double angVel;
        double vel;

        if(simulation)
        {
            Mat image = Mat::zeros(480, 640, CV_32F);
            cvtColor(image, image, CV_GRAY2BGR);
            circle(image, q, 8, Scalar(255,0,0), 2);
            line(image, Point2f(0,480), Point2f(100,480), Scalar(255,0,0), 10);
            line(image, Point2f(0,480), Point2f(0,380), Scalar(0,0,255), 10);


            angle = -atan2((q.y - (r->getY() + 50*sin(r->getH()))), (q.x - (r->getX() + 50*cos(r->getH())))) + r->getH();
            dist = sqrt((q.x - (r->getX() + 50*cos(r->getH())))*(q.x - (r->getX() + 50*cos(r->getH()))) + (q.y - (r->getY() + 50*sin(r->getH())))*(q.y - (r->getY() + 50*sin(r->getH()))));
            r->normalizeAngle(angle);

            angVel = angleController.calculate(0.0, angle, sample);
            vel = distanceController.calculate(-40.0, -dist, sample);

            r->setVel(vel);
            r->setAngVel(angVel);
            r->computeH(sample);
            r->computePos(sample);
            r->drawRobot(image, dist, angle);

            q.x += q_x;
            q.y += q_y;
            if(q.x > 640 || q.x < 0) q_x = -q_x;
            if(q.y > 480 || q.y < 0) q_y = -q_y;

            imshow("image", image);
        }
        else
        {
            //cout << "Uhel: "<<angle*180/3.14 << endl;
            //cout<<"Vzdlenost: "<<dist << endl;
            angVel = angleController.calculate(0.0, angle, sample);

            std::cout << "Distance: " << dist << std::endl;
            std::cout << "Angle: " << angle << std::endl;
            vel = distanceController.calculate(-40.0, -dist, sample);
            r->move(vel, angVel);
            imshow("out", imcopy);
            //imshow("input", frame);
        }
        if(waitKey(50) >= 0) break;
    }

}



