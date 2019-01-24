#include "Point.hpp"
#include <cmath>
#include "setup.hpp"
Point::Point() { x = y = 0; }
Point::Point(double x, double y) {
    this->x = x;
    this->y = y;
}
void Point::set(double x, double y) {
    this->x = x;
    this->y = y;
}
Point operator+(const Point& p1, const Point& p2) { return Point(p1.x + p2.x, p1.y + p2.y); }
Point operator-(const Point& p1, const Point& p2) { return Point(p1.x - p2.x, p1.y - p2.y); }
double operator*(const Point& p1, const Point& p2) { return p1.x * p2.x + p1.y * p2.y; }
Point operator*(const double& d, const Point& p) { return Point(d * p.x, d * p.y); }
bool operator<(const Point& p1, const Point& p2) {
    Point p1RotCCW = p1.rotate(1);
    if (p1RotCCW * p2 > 0.001) return true;
    return false;
}
bool operator>(const Point& p1, const Point& p2) {
    Point p1RotCCW = p1.rotate(1);
    if (p1RotCCW * p2 < -0.001) return true;
    return false;
}
Point Point::noZeroes() {
    if (this->x == 0.0) this->x = 0.000001;
    if (this->y == 0.0) this->y = 0.000001;
    return (*this);
}
double Point::magCross(const Point& p) { return fabs(this->x * p.y - this->y * p.x); }
double Point::mag() const { return sqrt(pow(x, 2.0) + pow(y, 2.0)); }
Point Point::rotate(int dir) const {
    if (dir > 0) {
        return Point(-y, x);
    } else {
        return Point(y, -x);
    }
}
Point Point::abs() {
    Point p(fabs(x), fabs(y));
    return p;
}
Point Point::unit() {
    double m = this->mag();
    return Point(this->x / m, this->y / m);
}
double Point::angleBetween(const Point& p) {
    double divisor = (this->mag() * p.mag());
    if (divisor == 0.0) divisor = 0.000001;
    return acos(clamp(((*this) * p) / divisor, -1.0, 1.0));
}