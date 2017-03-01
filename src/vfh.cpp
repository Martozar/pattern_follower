#include <iostream>
#include <pattern_follower/vfh.h>
void VFH::findCandidates(const std::vector<int> &bins) {
  candidateValleys.clear();
  std::vector<int> candidate;
  for (int i = 0; i < bins.size(); i++) {

    if (bins[i] < threshold) {
      candidate.push_back(i);
    } else {
      if (candidate.size() != 0) {
        candidateValleys.push_back(candidate);
        candidate.clear();
      }
    }
  }
  if (candidateValleys.size() == 0)
    candidateValleys.push_back(candidate);
}

int VFH::closestCandidate(const double &dist) {
  int candidate = 0;
  double min = 10000.;
  for (unsigned int i = 0; i < candidateValleys.size(); i++) {
    double back = std::fabs(candidateValleys[i].back() * alpha - dist);
    double front = std::fabs(candidateValleys[i].front() * alpha - dist);
    if (back < min || front < min) {
      candidate = i;
      min = std::min(back, front);
    }
  }
  return candidate;
}

double VFH::steeringDirection(const double &dist, const int &candidate) {

  auto candidateValley = candidateValleys[candidate];
  int candidateSize = candidateValley.size();
  double new_min = 0;
  double min = 10000.;
  int nearest = 0;
  int furtherst = 0;

  for (int i = 0; i < candidateSize; i++) {
    new_min = std::fabs(candidateValley[i] * alpha - dist);
    if (new_min > min)
      break;
    if (new_min < min) {
      nearest = i;
      min = new_min;
    }
  }
  double steeringDir = 0;
  if (nearest < candidateSize / 2) {
    furtherst = std::min(candidateSize - 1, nearest + maxSize);
  } else {
    furtherst = std::max(0, nearest - maxSize);
  }
  steeringDir =
      alpha * (candidateValley[nearest] + candidateValley[furtherst]) / 2;

  /*if (candidateSize < maxSize) {
    steeringDir =
        ((double)(candidateValley[0] + candidateValley.back())
  * alpha) / 2; } else { double right =
  (candidateValley.back() - maxSize / 2) * alpha; double left
  = (candidateValley[0] + maxSize / 2) * alpha; std::cout <<
  "Right: " << right << " Left: " << left << "\n"; if (dist <=
  right && dist >= left) steeringDir = dist; else { if
  (std::fabs(dist - right) > std::fabs(dist - left))
        steeringDir = left;
      else
      steeringDir = right;
    }
  }*/
  std::cout << "Target: " << dist << " Steer: " << steeringDir << "\n";
  return steeringDir;
}

double VFH::avoidObstacle(const std::vector<int> &bins, const double &dist) {

  double distDeg = radToDeg(dist, true);
  findCandidates(bins);
  int candidate = closestCandidate(distDeg);
  double steer = steeringDirection(distDeg, candidate);
  if (steer <= 2 * M_PI && steer >= M_PI)
    steer -= 2 * M_PI;
  steer = degToRad(steer);

  return steer;
}
