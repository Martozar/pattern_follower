#include <iostream>
#include <pattern_follower/vfh.h>

VFH::VFH(const int &mapSize, const int &resolution, const double &robRadius,
         const double &safety, const double &threshLow,
         const double &threshHigh, const double &densityB,
         const int &histRadius, const int &alpha, const int &maxSize,
         const double &mu1, const double &mu2, const double &mu3)
    : map_(new Map(mapSize, resolution)),
      histogram_(new Histogram(threshLow, threshHigh, densityB, mapSize / 2,
                               alpha, histRadius)) {
  maxSize_ = maxSize;
  alpha_ = alpha;
  mu1_ = mu1;
  mu2_ = mu2;
  mu3_ = mu3;
  prevOrient_ = 0;
  bins_ = 360 / alpha_;
  map_->init();
}

void VFH::findCandidates(const std::vector<int> &bins) {
  candidateValleys_.clear();
  std::vector<int> candidate;
  int start = -1;
  for (int i = 0; i < bins_ / 2; i++) {
    if (bins[i] == 1) {
      start = i;
      break;
    }
  }
  if (start == -1) {
    candidate.push_back(0);
    candidate.push_back(bins_ / 2);
    candidateValleys_.push_back(candidate);
    return;
  }
  bool left = true;
  for (int i = start; i <= (start + bins_); i++) {
    int mod = i % bins_;
    if (bins[mod] == 0 && left) {
      candidate.push_back(mod);
      left = false;
    }

    if (bins[mod] == 1 && !left) {
      candidate.push_back(mod - 1);
      if (candidate.back() < 0) {
        candidate.back() += bins_;
      }
      candidateValleys_.push_back(candidate);
      candidate.clear();
      left = true;
    }
  }
}

double VFH::costFunction(const int &c1, const int &c2) {
  int diff = c1 - c2;
  return std::min(std::min(std::fabs(diff), std::fabs(diff - bins_)),
                  std::fabs(diff + bins_));
}

double VFH::calculateCost(const int &candidate, const int &target,
                          const int &curHead) {
  double cost = 0;

  cost += mu1_ * costFunction(candidate, target);

  cost += mu2_ * costFunction(candidate, curHead);

  cost += mu3_ * costFunction(candidate, prevOrient_);

  return cost;
}

double VFH::chooseCandidate(const double &dist, const double &curHead) {

  int target = radCoordToDegCoord(dist) / (double)alpha_;
  int head = radCoordToDegCoord(curHead) / (double)alpha_;

  int candidate = 0;
  double min = 1e6;
  for (int i = 0; i < candidateValleys_.size(); i++) {
    int first = candidateValleys_[i].front();
    int second = candidateValleys_[i].back();
    int diff = delta(first, second);

    if (std::fabs(diff) < 2) {
      continue;
    }
    if (std::fabs(diff) < maxSize_) {
      double narrow = (first + second) / 2.0;
      double cost = calculateCost(candidate, target, head);
      if (cost < min) {
        candidate = narrow;
        min = cost;
      }
    } else {
      std::vector<double> cands;
      double center = (first + second) / 2;
      double left = (first + maxSize_ / 2) % (bins_);
      double right = (second - maxSize_ / 2);
      right += right < 0 ? (bins_) : 0;

      cands.push_back(center);
      cands.push_back(left);
      cands.push_back(right);
      if (target <= right && target >= left)
        cands.push_back(target);

      for (auto c : cands) {
        double cost = calculateCost(c, target, head);
        if (cost < min) {
          candidate = c;
          min = cost;
        }
      }
    }
  }
  return candidate;
}

int VFH::delta(const int &c1, const int &c2) {
  int delta = c1 - c2;
  if (delta > bins_ / 2) {
    delta -= bins_;
  } else if (delta < -(bins_ / 2)) {
    delta += bins_;
  }
  return delta;
}

double VFH::avoidObstacle(const std::vector<cv::Point2d> &points,
                          const cv::Point2d &target, const double &curHead) {
  double dist = target.y;
  map_->update(points, target);
  // map_->show();
  histogram_->update(map_->getMap());
  findCandidates(histogram_->getDensities());
  int candidate = chooseCandidate(dist, curHead);
  double ret = (degToRad(candidate * alpha_));
  ret -= ret > 3.0 * M_PI / 2.0 ? 5.0 * M_PI / 2.0 : M_PI / 2.0;
  /*ret -= (ret <= M_PI ? 0 : 2 * M_PI);*/
  return ret;
}
