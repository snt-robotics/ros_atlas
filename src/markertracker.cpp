#include "markertracker.h"

MarkerTracker::MarkerTracker()
{
}

void MarkerTracker::feed(int id, const tf::Transform& transf)
{
    Marker& marker = m_markers[id];
    marker.transfQueue.push_back(transf);
}

void MarkerTracker::guessAll()
{
    for (auto&& keyval : m_markers)
    {
        keyval.second.transf = averageTransforms(keyval.second.transfQueue);
        keyval.second.transfQueue.clear();
    }
}

std::vector<int> MarkerTracker::getKnownMarkers() const
{
    std::vector<int> markerIds;

    for (auto&& keyval : m_markers)
    {
        markerIds.push_back(keyval.first);
    }

    return markerIds;
}

tf::Transform MarkerTracker::getTransform(int id) const
{
    auto itr = m_markers.find(id);

    if (itr != m_markers.end())
        return itr->second.transf;

    return tf::Transform();
}

tf::Transform MarkerTracker::averageTransforms(const std::vector<tf::Transform>& transforms)
{
    if (transforms.empty())
        return tf::Transform();

    tf::Vector3 avrPos;
    tf::Quaternion avrQuad = transforms.front().getRotation();

    for (const tf::Transform& transf : transforms)
    {
        avrPos += transf.getOrigin();
        avrQuad.slerp(transf.getRotation(), 0.5);
    }

    avrPos /= transforms.size();

    return tf::Transform(avrQuad, avrPos);
}
