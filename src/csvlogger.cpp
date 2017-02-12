#include "csvlogger.h"

CSVLogger::CSVLogger()
{
}

void CSVLogger::log(std::string source, int id, float distance, const tf::StampedTransform& transf)
{
    std::replace(source.begin(), source.end(), '/', '_');

    auto itr = m_streams.find(source);

    if (itr == m_streams.end())
    {
        std::cout << "Open log: "
                  << "home/paul/" + source + ".csv" << std::endl;
        m_streams[source].open("/home/paul/" + source + ".csv", std::ofstream::out);
    }

    auto pos  = transf.getOrigin(); // std::setprecision(std::numeric_limits<double>::digits10)
    auto quad = transf.getRotation();
    m_streams[source] << source << ";" //
                      << id << ";" //
                      << transf.stamp_.toNSec() << ";" //
                      << pos.x() << ";" //
                      << pos.y() << ";" //
                      << pos.z() << ";" //
                      << quad.x() << ";" //
                      << quad.y() << ";" //
                      << quad.z() << ";" //
                      << quad.w() << ";" //
                      << distance //
                      << '\n'; //
}
