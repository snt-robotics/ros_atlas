#pragma once

#include <tf/LinearMath/Transform.h>

#include <map>
#include <vector>

class MarkerTracker
{
    struct Marker
    {
        // the final estimated transform
        tf::Transform transf;

        // a bunch of guesses
        std::vector<tf::Transform> transfQueue;
    };

public:
    MarkerTracker();

    void feed(int id, const tf::Transform& transf);
    void guessAll();

    std::vector<int> getKnownMarkers() const;
    tf::Transform getTransform(int id) const;

protected:
    tf::Transform averageTransforms(const std::vector<tf::Transform>& transforms);

private:
    std::map<int, Marker> m_markers;
};
