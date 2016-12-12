#include <pattern_follower/pattern.h>


Pattern::Pattern(double _size) : size(_size)
{
    confidence = -1;
    id = -1;
    orientation = -1;
    rotationVec = Mat::zeros(3,1,CV_32F);
    transitionVec = Mat::zeros(3,1,CV_32F);
    rotationMat = Mat::eye(3,3, CV_32F);
}

void Pattern::rotationMatrix(const Mat & rot_vec, Mat & rot_mat)
{
    Rodrigues(rot_vec, rot_mat);
}

void Pattern::getExtrinsics(int patternSize, const Mat & cameraMatrix, const Mat & distortions)
{

    std::vector<Point2f> pat2DPts;
    for(int i = 0; i < vertices.size(); i++)
    {
        pat2DPts.push_back(Point2f(vertices.at(i).x, vertices.at(i).y));
    }

    std::vector<Point3f> pat3DPts;
    pat3DPts.push_back(Point3f(0,0,0));
    pat3DPts.push_back(Point3f(patternSize,0,0));
    pat3DPts.push_back(Point3f(patternSize,patternSize,0));
    pat3DPts.push_back(Point3f(0,patternSize,0));

    Mat objectPts(pat3DPts);
    Mat imagePts(pat2DPts);

    solvePnP(pat3DPts, pat2DPts, cameraMatrix, distortions, rotationVec, transitionVec, CV_ITERATIVE);
}

void Pattern::draw(Mat & frame, const Mat & camMatrix, const Mat & distMatrix)
{
    Mat modelPts = (Mat_<float>(8,3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
                    0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size );
    std::vector<cv::Point2f> model2ImagePts;


    projectPoints(modelPts, rotationVec, transitionVec, camMatrix, distMatrix, model2ImagePts);

    cv::Point2f d;
    double distance = 0;
    for (int i =0; i<4; i++)
    {

        cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), cvScalar(0,255,255), 3);

    }

    model2ImagePts.clear();
}
