#include <iostream>
#include <pattern_follower/vfh.h>

VFH::VFH(const double &threshLow, const double &threshHigh,
         const double &robRadius, const double &densityB, const int &mapSize,
         const int &resolution, const double &safety, const int &histRadius,
         const int &maxSize, const int &alpha, const double &mu1,
         const double &mu2, const double &mu3)
    : map_(new Map(mapSize, resolution)),
      histogram_(new Histogram(threshLow, threshHigh, robRadius, mapSize / 2,
                               densityB, safety, alpha, histRadius)) {
  maxSize_ = maxSize;
  alpha_ = alpha;
  mu1_ = mu1;
  mu2_ = mu2;
  mu3_ = mu3;
  prevOrient_ = 0;
  map_->init();
}

void VFH::findCandidates(const std::vector<int> &bins) {
  candidateValleys_.clear();
  std::vector<int> candidate;
  int histSize = bins.size() - 1;
  for (int i = 0; i <= histSize; i++) {

    if (!bins[i]) {
      candidate.push_back(i);
    }
    if (bins[i] || i == histSize) {
      if (candidate.size() != 0) {
        // if we are at the end of hist
        // and its last and first cells are 0
        // it is a circle
        if (candidate.size() != histSize + 1 && i == histSize && !bins[0]) {
          candidate.insert(candidate.end(), candidateValleys_[0].begin(),
                           candidateValleys_[0].end());
        }
        candidateValleys_.push_back(candidate);
        candidate.clear();
      }
    }
  }
}

double VFH::costFunction(const int &c1, const int &c2) {
  int diff = c1 - c2;
  int constant = 360 / alpha_;
  return std::min(std::min(std::fabs(diff), std::fabs(diff - constant)),
                  std::fabs(diff + constant));
}
double VFH::calculateCost(const int &candidate, const int &target,
                          const int &curHead) {
  double cost = 0;

  cost += mu1_ * costFunction(candidate, target);

  cost += mu2_ * costFunction(candidate, curHead);

  cost += mu3_ * costFunction(candidate, prevOrient_);

  return cost;
}

int VFH::chooseCandidate(const double &dist, const double &curHead) {

  int target = radToDeg(dist) / alpha_;
  int head = radToDeg(curHead) / alpha_;

  int candidate = 0;
  double minCost = 10000.0;
  for (int i = 0; i < candidateValleys_.size(); i++) {
    if (candidateValleys_[i].size() < maxSize_) {
      int cNar =
          (candidateValleys_[i].front() + candidateValleys_[i].back()) / 2;
      minCost = calculateCost(cNar, target, head);
      candidate = cNar;
    } else {
      int cRight = candidateValleys_[i].front() + maxSize_ / 2;
      int cLeft = candidateValleys_[i].back() - maxSize_ / 2;

      if (target <= cLeft && target >= cRight) {
        // if target is choosen, its cost is 0
        candidate = target;
        break;
      }

      double costRight = calculateCost(cRight, target, head);
      double costLeft = calculateCost(cLeft, target, head);
      if (costRight < minCost || costLeft < minCost) {
        if (costRight < costLeft) {
          minCost = costRight;
          candidate = cRight;
        } else {
          minCost = costLeft;
          candidate = cLeft;
        }
      }
    }
  }
  std::cout << target << " " << candidate << "\n";
  prevOrient_ = candidate;
  return candidate;
}

double VFH::avoidObstacle(const std::vector<cv::Point2d> &points,
                          const cv::Point2d &target, const double &curHead) {
  double dist = target.y;
  map_->update(points, target);
  histogram_->update(map_->getMap());
  findCandidates(histogram_->getDensities());
  int candidate = chooseCandidate(dist, curHead);
  return degToRad(candidate * alpha_ - 180.0);
}
