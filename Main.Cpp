// Manipulator Code.cpp : This file contains the 'main' function. Program execution begins and ends there.
//This program calculates the optimal member lengths based on average torque for a 3 member arm given 3 set points the arm has to reach.

#include <iostream>
#include <cmath>

using namespace std;

const double grav = 9.81;

const double DegtoRad = 0.017453;
const double RadtoDeg = 57.2957795;
const double M_PI = 3.14159;

bool Invalid = true;

class Circle
{
public:
    double r;
    double x;
    double y;

    double minBetween2Circles(Circle c2, double r1, double r2) //calculates the minimum distance between two circles
    {
        double dis = sqrt((c2.x - x) * (c2.x - x) + (c2.y - y) * (c2.y - y)) - (r + c2.r);
        if (dis < 0)
        {
            dis = 0;
        }
        if (dis < (1 - (r1 + r2)))
            return (1 - (r1 + r2));
        else
            return dis;
    }


    double maxBetween2Circles(Circle c2) //calculates the maximum distance between two circles
    {
        return sqrt((c2.x - x) * (c2.x - x) + (c2.y - y) * (c2.y - y)) + (r + c2.r);
    }

};

class Member
{
public:

    double q1a, q1b, q2a, q2b;

    double l1, l2, l3;
    
    double q11a, q12a, q11b, q12b, q11, q12;
    double q13 = ( - 60.0 * DegtoRad);
    double x1 = 0.75;
    double y1 = 0.1;

    double q21a, q22a, q21b, q22b, q21, q22;
    double q23 = 0.0;
    double x2 = 0.5;
    double y2 = 0.5;

    double q31a, q32a, q31b, q32b, q31, q32;
    double q33 = (45.0*DegtoRad);
    double x3 = 0.2;
    double y3 = 0.6;

    double FinalT = 1000.0;

    void anglesReq(double PointX, double PointY, double Angle3) //Written by Brendan Chharawala
    {
        double member1 = l1;
        double member2 = l2;

        double PX = PointX - l3 * cos(Angle3);
        double PY = PointY - l3 * sin(Angle3);

        double required = pow(pow(PX, 2) + pow(PY, 2), 0.5);
        //cout << "length required is:" << required << endl;
        // Calculate angle from triangle to ground
        double angleOfTriToGroundDeg = atan(PY / PX);
        if (PX < 0) {
            angleOfTriToGroundDeg = angleOfTriToGroundDeg + M_PI;
        }
        //cout << "Angle to ground is: " << angleOfTriToGroundDeg*radTodegConv << endl;

        double A, B, C;
        //A = acos((member2 * member2 + required * required - member1 * member1) / (2 * member2 * required));
        B = acos((member1 * member1 + required * required - member2 * member2) / (2 * member1 * required));
        //C = acos((member1 * member1 + member2 * member2 - required * required) / (2 * member1 * member2));
        //cout <<  "Internal tri angle: " << B*radTodegConv << endl;

        q1a = (angleOfTriToGroundDeg + B);
        q1b = (angleOfTriToGroundDeg - B);
        //cout << "a1c1: " << result.angle1c1*radTodegConv << endl;
        //cout << "a1c2: " << result.angle1c2*radTodegConv << endl;

        double xMidway1 = member1 * cos(q1a);
        double yMidway1 = member1 * sin(q1a);
        //cout << "XMID1: " << xMidway1 << " YMID1: " << yMidway1 << endl;


        double xMidway2 = member1 * cos(q1b);
        double yMidway2 = member1 * sin(q1b);

        //cout << "XMID2: " << xMidway2 << " YMID2: " << yMidway2 << endl;

        double differenceX1 = PX - xMidway1;
        double differenceY1 = PY - yMidway1;
        double differenceX2 = PX - xMidway2;
        double differenceY2 = PY - yMidway2;
        //cout << "Case 1: dx: " << differenceX1 << " dy: " << differenceY1<< "Case 2: dx: " << differenceX2 << " dy: " << differenceY2 << endl;
        q2a = atan(differenceY1 / differenceX1);
        q2b = atan(differenceY2 / differenceX2);

        if (differenceX1 < 0 && differenceY1 < 0) {
            q2a = M_PI + q2a;
        }
        else if (differenceX1 < 0 && differenceY1 > 0) {
            q2a = M_PI + q2a;
        }
        else if (differenceX1 > 0 && differenceY1 < 0) {
            q2a = M_PI * 2 + q2a;
        }
        if (differenceX2 < 0 && differenceY2 < 0) {
            q2b = M_PI + q2b;
        }
        else if (differenceX2 < 0 && differenceY2 > 0) {
            q2b = M_PI + q2b;
        }
        else if (differenceX2 > 0 && differenceY2 < 0) {
            q2b = M_PI * 2 + q2b;
        }
    }

    void Angle() //Sets the angles in the member class
    {
        anglesReq(x1, y1, q13);
        q11a = q1a;
        q11b = q1b;
        q12a = q2a;
        q12b = q2b;

        anglesReq(x2, y2, q23);
        q21a = q1a;
        q21b = q1b;
        q22a = q2a;
        q22b = q2b;

        anglesReq(x3, y3, q33);
        q31a = q1a;
        q31b = q1b;
        q32a = q2a;
        q32b = q2b;
    }

    double tourqe(double q1, double q2, double q3) //calculates torque for one position
    {
        double T4 = (l1 * cos(q1) + (l2 * cos(q2)) + (l3 * cos(q3))) * grav * 5;
        double T3 = (l1 * cos(q1) + l2 * cos(q2) + l3 * cos(q3) / 2) * grav * l3;
        double T2 = (l1 * cos(q1) + l2 * cos(q2) / 2) * grav * 2 * l2;
        double T1 = (l1 * l1) * 2 * grav * cos(q1); 

        return T1 + T2 + T3 + T4;
    }

    void FinalTCalc() //does final torque calculation and determine the optimal for the double angle ones.
    {
        Angle();

        double T11a = tourqe(q11a, q12a, q13);
        double T22a = tourqe(q21a, q22a, q23);
        double T33a = tourqe(q31a, q32a, q33);

        double T11b = tourqe(q11b, q12b, q13);
        double T22b = tourqe(q21b, q22b, q23);
        double T33b = tourqe(q31b, q32b, q33);

        double T11;
        double T22;
        double T33;

        if (T11a > T11b)
        {
            T11 = T11b;
            q11 = q11b;
            q12 = q12b;
        }
        else
        {
            T11 = T11a;
            q11 = q11a;
            q12 = q12a;
        }
           
        if (T22a > T22b)
        {
            T22 = T22b;
            q21 = q21b;
            q22 = q22b;
        }
        else
        {
            T22 = T22a;
            q21 = q21a;
            q22 = q22a;
        }

        if (T33a > T33b)
        {
            T33 = T33b;
            q31 = q31b;
            q32 = q32b;
        }
        else
        {
            T33 = T33a;
            q31 = q31a;
            q32 = q32a;
        }

        FinalT = sqrt(T11 * T11 + T22 * T22 + T33 * T33);
    }

};


void printMemberDetails(Member minVal) //prints out member details
{
    cout << "Final Torque: " << minVal.FinalT << '\n';
    cout << "Final Torque 1: " << minVal.tourqe(minVal.q11, minVal.q12, minVal.q13) << ", Length 1: " << minVal.l1 << ", Angle 1: " << minVal.q11 * RadtoDeg << ", Length 2: " << minVal.l2 << ", Angle 2: " << minVal.q12 * RadtoDeg << ", Length 3: " << minVal.l3 << ", Angle 3: " << minVal.q13 * RadtoDeg << '\n';

    cout << "Final Torque 2: " << minVal.tourqe(minVal.q21, minVal.q22, minVal.q23) << ", Length 1: " << minVal.l1 << ", Angle 1: " << minVal.q21 * RadtoDeg << ", Length 2: " << minVal.l2 << ", Angle 2: " << minVal.q22 * RadtoDeg << ", Length 3: " << minVal.l3 << ", Angle 3: " << minVal.q23 * RadtoDeg << '\n';

    cout << "Final Torque 3: " << minVal.tourqe(minVal.q31, minVal.q32, minVal.q33) << ", Length 1: " << minVal.l1 << ", Angle 1: " << minVal.q31 * RadtoDeg << ", Length 2: " << minVal.l2 << ", Angle 2: " << minVal.q32 * RadtoDeg << ", Length 3: " << minVal.l3 << ", Angle 3: " << minVal.q33 * RadtoDeg << '\n';
}

bool checkIntersect(Member temp, double q1, double q2, double q3) //checks to ensure the members do not cross each other
{
    double x1, y1, x2, y2, x3, y3;
    x1 = 0.0 + temp.l1 * cos(q1);
    y1 = 0.0 + temp.l1 * sin(q1);

    x2 = x1 + temp.l2 * cos(q2);
    y2 = y1 + temp.l2 * sin(q2);

    x3 = x2 + temp.l3 * cos(q3);
    y3 = y2 + temp.l3 * sin(q3);

    // Find the orientation of triplet (0.0, pt1, pt2)
    double orientation1 = (y1 - 0.0) * (x2 - x1) -
        (x1 - 0.0) * (y2 - y1);

    // Find the orientation of triplet (0.0, pt1, pt3)
    double orientation2 = (y1 - 0.0) * (x3 - x1) -
        (x1 - 0.0) * (y3 - y1);

    // Find the orientation of triplet (pt2, pt3, 0.0)
    double orientation3 = (y3 - y2) * (0.0 - x3) -
        (x3 - x2) * (0.0 - y3);

    // Find the orientation of triplet (pt2, pt3, pt1)
    double orientation4 = (y3 - y2) * (x1 - x3) -
        (x3 - x2) * (y1 - y3);

    // General case: The lines intersect if orientations are different
    if (orientation1 * orientation2 < 0 && orientation3 * orientation4 < 0) {
        return true;
    }

    // Special case: The lines are collinear if all orientations are 0
    if (orientation1 == 0 && orientation2 == 0 && orientation3 == 0 && orientation4 == 0) {
        // Check if the line segments overlap
        if (.0 <= x3 && x3 <= x1 && x2 <= x3 && x3 <= x3) {
            return true;
        }
        if (x3 <= 0.0 && 0.0 <= x1 && x3 <= x2 && x2 <= x1) {
            return true;
        }
    }

    // The lines do not intersect
    return false;
}

void Circles(double maxR, double RadiusInterval, double LengthInterval) //Iterates through to find the correct values
{
    Circle c1;
    c1.x = 0.75;
    c1.y = 0.1;
    Circle c2;
    c2.x = 0.5;
    c2.y = 0.5;
    Circle c3;
    c3.x = 0.2;
    c3.y = 0.6;

    Circle origin;
    origin.x = 0;
    origin.y = 0;

    Member minVal;

    for (origin.r = RadiusInterval; origin.r < maxR; origin.r = origin.r + RadiusInterval) //increments origin circle
    {
        cout << "Incremented Origin Radius " << origin.r << '\n';

        for (c1.r = RadiusInterval; c1.r < maxR; c1.r = c1.r + RadiusInterval) //increments point circles
        {
            cout << "Incremented Point Radius " << c1.r << '\n';

            c2.r = c1.r;
            c3.r = c1.r;

            Member temp;
            temp.l1 = origin.r;
            temp.l3 = c1.r;

            double minDist = origin.minBetween2Circles(c1, origin.r, c1.r);
            double maxDist = origin.maxBetween2Circles(c1);

            for (temp.l2 = minDist; temp.l2 < maxDist; temp.l2 = temp.l2 + LengthInterval) //increments l2
            {
                temp.FinalTCalc();
                if (temp.FinalT < minVal.FinalT && temp.q11 < M_PI && temp.q21 < M_PI && temp.q31 < M_PI)
                {
                    if (!checkIntersect(temp, temp.q11, temp.q12, temp.q13) && !checkIntersect(temp, temp.q21, temp.q22, temp.q23) && !checkIntersect(temp, temp.q31, temp.q32, temp.q33))
                    {
                        minVal = temp;
                        cout << "New Min Torque is: " << minVal.FinalT << '\n';
                        printMemberDetails(minVal);
                    }
                }
            }
        }
    }
}

int main()
{

    cout << "STARTING" << '\n';

    Circles(3.0, 0.1, 0.1);

    cout << "FINISHED" << '\n';
    return EXIT_SUCCESS;
}

