#include <coordinates.hpp>

using namespace Eigen;
using namespace robot_tweezers;


Coordinates::Coordinates() { }

Coordinates::Coordinates(Matrix3f frame, Vector3f origin)
{
    this->frame = frame;
    this->origin = origin;
}

Coordinates::Coordinates(Matrix4f coordinates)
{
    setCoordinates(coordinates);
}

void Coordinates::setCoordinates(Matrix4f coordinates)
{
    for (int i = 0; i < 3; i++)
    {
        this->origin(i) = coordinates(i, 3);
        for (int j = 0; j < 3; j++)
        {
            this->frame(i, j) = coordinates(i, j);
        }
    }
}

Matrix4f Coordinates::getCoordinates()
{
    Matrix4f matrix;
    matrix << 
        frame, origin,
        0, 0, 0, 1;
    return matrix;
}