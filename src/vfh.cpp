#include <iostream>
#include <pattern_follower/vfh.h>

VFH::VFH(const int &mapSize, const int &resolution, const double &robRadius,
         const double &safety, const double &threshLow,
         const double &threshHigh, const double &densityB,
         const int &histRadius, const int &alpha, const int &maxSize,
         const double &mu1, const double &mu2, const double &mu3)
    : map_(new Map(mapSize, resolution)),
      histogram_(
          new Histogram(threshLow, threshHigh, densityB, alpha, histRadius)) {
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
  for (int i = 0; i < bins_; i++) {
    if (bins[i] == 1) {
      start = i;
      break;
    }
  }
  if (start == -1) {
    candidate.push_back(0);
    candidate.push_back(bins_ - 1);
    candidateValleys_.push_back(candidate);
    return;
  }
  bool left = true;
  for (int i = start; i <= (start + bins_); i++) {
    int mod = i % bins_;
    if (bins[mod] == 0 && left) {
      candidate.push_back(bins[mod]);
      left = false;
    }

    if (bins[mod] == 1 && !left) {
      candidate.push_back(bins[mod - 1]);
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
  // TODO has to be donnnnne
  std::cout << dist << "\n";
  int target = (radToDeg(dist) + 180) / alpha_;
  std::cout << target << "\n";
  int head = (radToDeg(curHead) + 180) / alpha_;
  int candidate = 0;
  for (int i = 0; i < candidateValleys_.size(); i++) {
    int first = candidateValleys_[i].front();
    int second = candidateValleys_[i].back();
    int diff = delta(first, second);

    if (std::fabs(diff) < 2) {
      continue;
    }
    if (std::fabs(diff) < maxSize_) {
      double narrow = (first + second) / 2.0;
    } else {
      double center = (first + second) / 2.0;
      double left = (first + maxSize_ / 2) % (bins_);
      double right = (second - maxSize_ / 2);
      right += right < 0 ? (bins_) : 0;
      if (target >= right && target <= left) {
        // target
      }
    }
  }
  return candidate;
}

int VFH::delta(const int &c1, const int &c2) {
  int delta = c1 - c2;
  if (delta > bins_ / 2) {
    delta -= bins_;
  } else if (delta < -180) {
    delta += bins_;
  }
  return delta;
}

double VFH::avoidObstacle(const std::vector<cv::Point2d> &points,
                          const cv::Point2d &target, const double &curHead) {
  double dist = target.y;
  map_->update(points, target);
  map_->show();
  histogram_->update(map_->getMap());
  findCandidates(histogram_->getDensities());
  int candidate = chooseCandidate(dist, curHead) - bins_ / 2;
  double ret = (degToRad(candidate * alpha_));
  std::cout << "Pred " << ret << "\n";
  ret -= ret > M_PI ? 2 * M_PI : 0;
  std::cout << "Po " << ret << "\n";
  /*ret -= (ret <= M_PI ? 0 : 2 * M_PI);*/
  return ret;
}
