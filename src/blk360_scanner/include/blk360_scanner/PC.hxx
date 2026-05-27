#pragma once

#include "BLK360.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <cmath>
#include <fstream>
#include <string>
#include <iostream>

namespace pointcloud
{
    namespace
    {
        constexpr char DELIMITER = ',';

        struct CartesianPoint
        {
            float x, y, z, intensity;
        };

        struct Color
        {
            float r, g, b;
        };

        struct PolarPoint
        {
            float hAngle, vAngle, distance, intensity;

            CartesianPoint toCartesian() const
            {
                CartesianPoint cartesian;

                cartesian.x = -distance * std::sin(vAngle) * std::sin(hAngle);
                cartesian.y = -distance * std::cos(vAngle);
                cartesian.z = -distance * std::sin(vAngle) * std::cos(hAngle);
                cartesian.intensity = intensity;

                return cartesian;
            }

            bool isInvalid() const
            {
                return std::abs(hAngle) <= std::numeric_limits<float>::epsilon()
                    && std::abs(vAngle) <= std::numeric_limits<float>::epsilon()
                    && std::abs(distance) <= std::numeric_limits<float>::epsilon()
                    && std::abs(intensity) <= std::numeric_limits<float>::epsilon();
            }
        };

        int convertColorComponent(float value)
        {
            return std::min(255, std::max(0, static_cast<int>(value * 255)));
        }
    }

    inline std::ofstream createOutputFile(const Blk360_scanId_t& scanId)
    {
        const std::string filename = "pointcloud_" + std::to_string(scanId) + ".csv";
        std::ofstream file(filename, std::ios_base::out);
        if (file.is_open())
        {
            // append csv header
            file << "x [m]" << DELIMITER << "y [m]" << DELIMITER << "z [m]" << DELIMITER << "r" << DELIMITER << "g" << DELIMITER << "b" << "\n";
        }
        else
        {
            std::cerr << "Could not open file: " << filename << std::endl;
        }
        return file;
    }

    inline void writeChunk(std::ofstream& stream, const char* const data, const std::size_t dataSize, const char* colors, const std::size_t colorsSize)
    {
        const std::size_t totalElements = dataSize / sizeof(PolarPoint);
        const std::size_t totalColors = colorsSize / sizeof(Color);

        assert(totalElements == totalColors);

        const PolarPoint* dataStart = reinterpret_cast<const PolarPoint*>(data);
        const PolarPoint* dataEnd = dataStart + totalElements;

        const Color* colorsStart = reinterpret_cast<const Color*>(colors);
        const Color* colorsEnd = colorsStart + totalColors;

        const PolarPoint* polarPoint = dataStart;
        const Color* pointColor = colorsStart;
        for (; polarPoint != dataEnd && pointColor != colorsEnd; polarPoint++, pointColor++)
        {
            if (polarPoint->isInvalid())
            {
                continue; // omit this point
            }

            const auto cartesianPoint = polarPoint->toCartesian();
            stream << cartesianPoint.x << DELIMITER << cartesianPoint.y << DELIMITER << cartesianPoint.z << DELIMITER;
            stream << convertColorComponent(pointColor->r) << DELIMITER
                   << convertColorComponent(pointColor->g) << DELIMITER
                   << convertColorComponent(pointColor->b) << '\n';
        }
    }
}
