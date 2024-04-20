#include "mppi/costmap.hpp"

namespace mppi{

Costmap::Costmap(double minXArg, double minYArg, double resolutionArg, int widthArg, int heightArg)
        : min_x{minXArg},
          min_y{minYArg},
          resolution{resolutionArg},
          width{widthArg},
          height{heightArg}
{
    max_x = min_x + width * resolution;
    max_y = min_y + height * resolution;
}
std::pair<int, int> Costmap::indexToRC(int index) const
{
    int c = index % width;
    int r = (index - c) / width;
    return std::pair<int, int>(r, c);
}

bool Costmap::inMap(double x, double y) const
{
    return (min_x < x) && (x < (min_x + width * resolution)) &&
            (min_y < y) && (y < (min_y + height * resolution));
}

int Costmap::xToCol(double x) const
{
    return floor((x - min_x) / resolution);
}

double Costmap::colToX(int col) const
{
    return min_x + col * resolution;
}

int Costmap::yToRow(double y) const
{

    return floor((y - min_y) / resolution);
}

double Costmap::rowToY(int row) const
{

    return min_y + row * resolution;
}

uint8_t Costmap::get(int r, int c) const
{
    if ((r < 0) || (height <= r) || (c < 0) || (width <= c))
    {
        std::cout << "INSIDE costmap get IF STATEMENT: " << std::endl;

        return 0;
    }

    // std::cout << "OUTSIDE costmap get IF STATEMENT: " << static_cast<int>(data[width * r + c]) << std::endl;

    // for (const auto& element : data) {
    //     std::cout << static_cast<int>(element) << std::endl;
    // }

    return data[width * r + c];
}

uint8_t Costmap::vget(double x, double y) const
{
    // std::cout << "Inside costmap vget: " << get(yToRow(y), xToCol(x)) << std::endl;

    return get(yToRow(y), xToCol(x));
}

}