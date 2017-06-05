/*
 * ATLAS - Cooperative sensing
 * Copyright (C) 2017  Paul KREMER
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
