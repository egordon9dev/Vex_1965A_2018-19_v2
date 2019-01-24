#ifndef POINT_H
#define POINT_H
class Point {
   public:
    double x, y;
    Point();
    Point(double x, double y);
    void set(double x, double y);
    friend Point operator+(const Point& p1, const Point& p2);
    friend Point operator-(const Point& p1, const Point& p2);
    friend double operator*(const Point& p1, const Point& p2);
    friend Point operator*(const double& d, const Point& p);
    friend bool operator>(const Point& p1, const Point& p2);
    friend bool operator<(const Point& p1, const Point& p2);
    Point noZeroes();
    double magCross(const Point& p);
    double mag() const;
    Point abs();
    Point unit();
    Point rotate(int dir) const;
    double angleBetween(const Point& p);
};
#endif