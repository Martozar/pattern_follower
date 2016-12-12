#include <pattern_follower/tamplatematcher.h>

bool TamplateMatcher::identifyPattern(const Mat & src, patInfo & out)
{
    double correlation;

    Mat inter = src(Range(normSize/4, 3*normSize/4), Range(normSize/4, 3*normSize/4));

    out.maxCor = -1.0;
    Mat output;
    for(int i = 0; i < library.size()/4; i++)
    {

        for(int j = 0; j < 4; j++)
        {
            matchTemplate(inter, library.at(i*4+j), output, TM_CCOEFF_NORMED );
            correlation = mean(output).val[0];

            if(correlation > out.maxCor)
            {
                out.maxCor = correlation;
                out.index = i+1;
                out.ori = j;
            }
        }
    }
    return out.maxCor > confThreshold;
}

void TamplateMatcher::detect(const Mat & src, const Mat & cameraMatrix, const Mat & distortions, std::vector<Point2f> & refinedVertices, const int & vertex, std::vector<Pattern> & foundPatterns)
{
    patInfo out;
    if(identifyPattern(src, out))
    {
        Pattern patCand;
        patCand.setID(out.index);
        patCand.setOrientation(out.ori);
        for(int i = 0; i < 4; i++)
        {
            patCand.getVetices().push_back(refinedVertices.at((8 - out.ori+vertex-i)%4));
        }

        patCand.getExtrinsics(patCand.getSize(), cameraMatrix, distortions);
        foundPatterns.push_back(patCand);
    }
}
