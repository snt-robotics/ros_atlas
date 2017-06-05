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

#include "helpers.h"

namespace tf2
{
// Print helpers
std::ostream& operator<<(std::ostream& os, const tf2::Vector3& vec)
{
    return os << "Vec3 [X:] " << vec.x() << " [Y:] " << vec.y() << "[Z:] " << vec.z();
}

std::ostream& operator<<(std::ostream& os, const tf2::Quaternion& quat)
{
    return os << "Quad [X:] " << quat.x() << " [Y:] " << quat.y() << "[Z:] " << quat.z() << "[W:] " << quat.w();
}

std::ostream& operator<<(std::ostream& os, const tf2::Transform& transform)
{
    return os << transform.getOrigin() << " " << transform.getRotation();
}
}

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
    return os << pose.pos << " " << pose.rot;
}
