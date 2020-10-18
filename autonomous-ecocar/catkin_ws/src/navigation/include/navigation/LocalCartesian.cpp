//
//  LocalCartesian.cpp
//  GPS2local
//
//  Created by Alessandra Bellina on 11/05/2017.
//  Copyright © 2017 Alessandra Bellina. All rights reserved.
//

#include "LocalCartesian.hpp"

#include <iostream>

PointCoordinates::PointCoordinates()
{
    c1 = 0;
    c2 = 0;
    c3 = 0;
};

PointCoordinates::PointCoordinates(double c1_val, double c2_val, double c3_val)
{
    c1 = c1_val;
    c2 = c2_val;
    c3 = c3_val;
};


PointCoordinates GPS_2_ECEF(PointCoordinates P0_GPS)
{
    PointCoordinates P0_ECEF;
    double lon = P0_GPS.c1;
    double lat = P0_GPS.c2;
    double height = P0_GPS.c3;
    
    double NE = REa/sqrt(1-e*e*sin(lat* PI / 180.0)*sin(lat* PI / 180.0));
    P0_ECEF.c1 = (NE+height)*cos(lat* PI / 180.0)*cos(lon* PI / 180.0);
    P0_ECEF.c2 = (NE+height)*cos(lat* PI / 180.0)*sin(lon* PI / 180.0);
    P0_ECEF.c3 = (NE*(1-e*e)+height)*sin(lat* PI / 180.0);

    return P0_ECEF;
};

PointCoordinates ECEF_2_NED(PointCoordinates P0_ECEF, PointCoordinates p_reference_GPS)
{
    PointCoordinates P0_NED;
    double lon_r = p_reference_GPS.c1;
    double lat_r = p_reference_GPS.c2;
    
    
    PointCoordinates p_reference_ECEF = GPS_2_ECEF(p_reference_GPS);

    PointCoordinates difference;
    difference.c1 = P0_ECEF.c1 - p_reference_ECEF.c1;
    difference.c2 = P0_ECEF.c2 - p_reference_ECEF.c2;
    difference.c3 = P0_ECEF.c3 - p_reference_ECEF.c3;
    
    double R[3][3];
    R[0][0] = - sin(lat_r* PI / 180.0)*cos(lon_r* PI / 180.0);
    R[0][1] = -sin(lat_r* PI / 180.0)*sin(lon_r* PI / 180.0);
    R[0][2] = cos(lat_r* PI / 180.0);
    R[1][0] = -sin(lon_r* PI / 180.0);
    R[1][1] = cos(lon_r* PI / 180.0);
    R[1][2] = 0;
    R[2][0] = -cos(lat_r* PI / 180.0)*cos(lon_r* PI / 180.0);
    R[2][1] = -cos(lat_r* PI / 180.0)*sin(lon_r* PI / 180.0);
    R[2][2] = -sin(lat_r* PI / 180.0);
    
    double p[3] = {difference.c1, difference.c2, difference.c3};
    double res[3] = {0,0,0};
    
    for( int i = 0; i < 3; ++i)
    {       for(int k = 0; k < 3; ++k)
            {
                res[i]=res[i] + R[i][k] * p[k];
            }
    }
    P0_NED.c1 = res[0];
    P0_NED.c2 = res[1];
    P0_NED.c3 = res[2];

    return P0_NED;
};

PointCoordinates NED_2_LOCAL(PointCoordinates P0_NED, double heading)
{
    // Heading: angle between NED_frame and local frame. Logal_frame obtained by rotating of heading counter-clockwise.
    //  Define transformation matrix
    
    PointCoordinates P0_LOCAL;
    double R[3][3];
    R[0][0] = cos(heading* PI / 180.0);
    R[0][1] = sin(heading* PI / 180.0);
    R[0][2] = 0;
    R[1][0] = -sin(heading* PI / 180.0);
    R[1][1] = cos(heading* PI / 180.0);
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 1;
    
    double p[3] = {P0_NED.c1, P0_NED.c2, P0_NED.c3};
    double res[3]= {0,0,0};
    
    for( int i = 0; i < 3; ++i)
        for(int k = 0; k < 3; ++k)
        {
            res[i]=res[i] + R[i][k] * p[k];
        }
    
    P0_LOCAL.c1 = res[0];
    P0_LOCAL.c2 = -res[1];
    P0_LOCAL.c3 = res[2];
    return P0_LOCAL;
    
};



PointCoordinates GPS_2_LOCAL(PointCoordinates P0_GPS, PointCoordinates p_reference_GPS, double heading)
{
    PointCoordinates P0_ECEF = GPS_2_ECEF(P0_GPS);
    PointCoordinates P0_NED = ECEF_2_NED(P0_ECEF, p_reference_GPS);
    PointCoordinates P0_LOCAL = NED_2_LOCAL(P0_NED, heading);

    return P0_LOCAL;
};

double getInitialHeading(PointCoordinates p0, PointCoordinates p1 )
{
    double lon0 = p0.c1*PI/180;
    double lat0 = p0.c2*PI/180;
    double lon1 = p1.c1*PI/180;
    double lat1 = p1.c2*PI/180;
    
    double X = cos(lat1)*sin(lon1-lon0);
    double Y = cos(lat0)*sin(lat1)-sin(lat0)*cos(lat1)*cos(lon1-lon0);
    
    return atan2(X,Y);
};

