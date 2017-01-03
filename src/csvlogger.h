#pragma once

#include <fstream>
#include <iostream>
#include <map>

#include <tf/transform_datatypes.h>

class CSVLogger
{
public:
    CSVLogger();

    void log(std::string source, int id, float distance, const tf::StampedTransform& transf);

private:
    std::map<std::string, std::ofstream> m_streams;
};
