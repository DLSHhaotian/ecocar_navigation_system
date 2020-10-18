//
//  LocalCartesian.hpp
//  GPS2local
//
//  Created by Alessandra Bellina on 11/05/2017.
//  Copyright © 2017 Alessandra Bellina. All rights reserved.
//

#ifndef LocalCartesian_hpp
#define LocalCartesian_hpp

#include <stdio.h>
#include <math.h>

// Some constants

const double REa = 6378137; //m
const double f = 1/298.257223563;
const double REb = REa*(1-f);
const double e = sqrt(REa*REa-REb*REb)/REa;
#define PI 3.14159265


//PointCoordinates class declaration

class PointCoordinates
{
public:
    double c1, c2, c3;
    //Constructor:
    PointCoordinates();
    PointCoordinates (double c1_val, double c2_val, double c3_val);
};

PointCoordinates GPS_2_ECEF(PointCoordinates P0_GPS);
PointCoordinates ECEF_2_NED(PointCoordinates P0_ECEF, PointCoordinates p_reference_GPS);
PointCoordinates NED_2_LOCAL(PointCoordinates P0_NED, double heading);
PointCoordinates GPS_2_LOCAL(PointCoordinates P0_GPS, PointCoordinates p_reference_GPS, double heading);
double getInitialHeading(PointCoordinates p0, PointCoordinates p1 );

#endif /* LocalCartesian_hpp */


