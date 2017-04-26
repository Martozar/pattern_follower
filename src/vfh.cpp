#include <iostream>
#include <pattern_follower/vfh.h>

VFH::VFH(const FileNode &fn)
    : map_(new Map(fn["Map"])), histogra(new Histogram(fn["Histogram"])) {
  maxSize_ = fn["max_narrow_size"];
  alpha_ = fn["Histogram"]["alpha"];
  mu1_ = fn["mu_1"];
  mu2_ = fn["mu_2"];
  mu3_ = fn["mu_3"];
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
      if (candidate[1] < 0 || candidate[0] > candidate[1]) {
        candidate[1] += bins_;
      }
      candidateValleys_.push_back(candidate);
      candidate.clear();
      left = true;
    }
  }
}

double VFH::costFunction(const int &c1, const int &c2) {
  int diff = delta(c1, c2);
  return std::abs(diff);
}

double VFH::calculateCost(const int &candidate, const int &target,
                          const int &curHead) {
  double cost = 0;

  cost += mu1_ * costFunction(target, candidate);

  cost += mu2_ * costFunction(candidate, curHead);

  cost += mu3_ * costFunction(prevOrient_, candidate);

  return cost;
}

double VFH::chooseCandidate(const double &dist, const double &curHead) {

  int target = radCoordToDegCoord(dist) / (double)alpha_;
  int head = radCoordToDegCoord(curHead) / (double)alpha_;

  double candidate = 0;
  double min = 1e6;

  std::vector<double> cands;
  for (int i = 0; i < candidateValleys_.size(); i++) {
    double first = candidateValleys_[i].front();
    double second = candidateValleys_[i].back();
    double diff = delta(first, second);

    if (std::fabs(diff) < 2) {
      continue;
    }
    if (std::fabs(diff) < maxSize_) {
      double narrow = (first + second) / 2.0;
      narrow = narrow >= bins_ ? narrow - bins_ : narrow;
      cands.push_back(narrow);
      std::cout << narrow << "\n";
    } else {
      double center = (first + second) / 2.0;
      center = center >= bins_ ? center - bins_ : center;
      double left = (first + (double)maxSize_ / 2.0);

      double right = (second - (double)maxSize_ / 2.0);
      left = left >= bins_ ? left - bins_ : left;
      right =
          right < 0 ? right + bins_ : right >= bins_ ? right - bins_ : right;

      cands.push_back(center);
      cands.push_back(left);
      cands.push_back(right);
      if (delta(target, left) < 0.0 && delta(target, right) > 0.0)
        cands.push_back(target);
    }
  }

  for (auto c : cands) {
    double cost = calculateCost(c, target, head);
    if (cost < min) {
      candidate = c;
      min = cost;
    }
  }
  prevOrient_ = candidate;
  return candidate;
}

int VFH::delta(const int &c1, const int &c2) {
  int delta = c2 - c1;
  if (delta > bins_ / 2) {
    delta -= bins_;
  } else if (delta < -(bins_ / 2)) {
    delta += bins_;
  }
  return delta;
}

double VFH::avoidObstacle(const std::vector<cv::Point2d> &points,
                          const cv::Point2d &target, const double &curHead,
                          double &speedRatio) {
  double dist = target.y;
  map_->update(points, target);
  // map_->show();
  histogram_->update(map_->getMap());
  findCandidates(histogram_->getDensities());
  int candidate = chooseCandidate(dist, curHead);
  speedRatio = histogram_->ratio(candidate);

  double ret = (degToRad(candidate * alpha_));
  ret -= ret > 3.0 * M_PI / 2.0 ? 5.0 * M_PI / 2.0 : M_PI / 2.0;
  return ret;
}
