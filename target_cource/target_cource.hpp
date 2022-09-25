#ifndef TARGET_COURSE_HPP
#define TARGET_COURSE_HPP

#include <vector>
#include <array>
#include <cmath>

class TargetCourse
{
private:
    int old_nearest_point_index_ = -1; 
public:
    std::vector<std::array<float, 2>> points_;
    TargetCourse();
    TargetCourse(std::vector<std::array<float, 2>> &points);
    ~TargetCourse();
    void SetPoints(std::vector<std::array<float, 2>> &points);
    int SearchTargetIndex(int x, int y, double v);
    std::array<int, 2> CalcTotalDistance(void);
    double Lfc_ = 300;
    double K_ = 100;
};

TargetCourse::TargetCourse() {
}

TargetCourse::TargetCourse(std::vector<std::array<float, 2>> &points)
{
    points_ = points;
}

TargetCourse::~TargetCourse()
{
}

void TargetCourse::SetPoints(std::vector<std::array<float, 2>> &points) {
    old_nearest_point_index_ = -1;
    points_ = points;
 
}

int TargetCourse::SearchTargetIndex(int x, int y, double v)
{
    int tmp_index;
    if (old_nearest_point_index_ == -1)
    {
        tmp_index = 0;
        double min_distance = __DBL_MAX__;
        for (const auto &point : points_)
        {
            int dx = x - point.at(0);
            int dy = y - point.at(1);
            double distance = hypot(dx, dy);
            if (distance < min_distance)
            {
                min_distance = distance;
                tmp_index = &point - &points_[0];
            }
        }
        old_nearest_point_index_ = tmp_index;
    }
    else
    {
        tmp_index = old_nearest_point_index_;
        double distance_this_index = hypot(x - points_.at(tmp_index).at(0), y - points_.at(tmp_index).at(1));

        for (int i = tmp_index + 1; i < (int)points_.size(); i++)
        {
            int dx = x - points_.at(i).at(0);
            int dy = y - points_.at(i).at(1);
            double distance = hypot(dx, dy);
            if (distance_this_index < distance)
            {
                break;
            }
            tmp_index = ((i + 1) < (int)points_.size()) ? (i + 1) : i;
            distance_this_index = distance;
        }
        old_nearest_point_index_ = tmp_index;
    }

    double look_forward_distance =  K_ + v * 0.001 * Lfc_; 
    while (look_forward_distance > hypot(x - points_.at(tmp_index).at(0), y - points_.at(tmp_index).at(1)))
    {
        if ((tmp_index + 1) >= (int)points_.size())
        {
            break;
        }
        tmp_index += 1;
    }
    return tmp_index;
}


std::array<int, 2> TargetCourse::CalcTotalDistance(void) {
    std::array<int, 2> res = {0, 0}; 
    for (int i = 1; i < (int)points_.size(); i++) {
        res.at(0) += abs(points_.at(i).at(0) - points_.at(i-1).at(0));
        res.at(1) += abs(points_.at(i).at(1) - points_.at(i-1).at(1));
    }
    return res;
}

#endif