#!/usr/bin/python

import random

import numpy as n

import sys
import os
import time

import shutil

## Convex hull stuff
def _angle_to_point(point, centre):
    '''calculate angle in 2-D between points and x axis'''
    delta = point - centre
    res = n.arctan(delta[1] / delta[0])
    if delta[0] < 0:
        res += n.pi
    return res

def area_of_triangle(p1, p2, p3):
    '''calculate area of any triangle given co-ordinates of the corners'''
    return n.linalg.norm(n.cross((p2 - p1), (p3 - p1)))/2.

def convex_hull(points):
    '''Calculate subset of points that make a convex hull around points

    Recursively eliminates points that lie inside two neighbouring points until only convex hull is remaining.

    :Parameters:
        points : ndarray (2 x m)
        array of points for which to find hull

    :Returns:
        hull_points : ndarray (2 x n)
        convex hull surrounding points
    '''

    n_pts = points.shape[1]
    assert(n_pts > 5)
    centre = points.mean(1)

    angles = n.apply_along_axis(_angle_to_point, 0, points, centre)
    pts_ord = points[:,angles.argsort()]

    pts = [x[0] for x in zip(pts_ord.transpose())]
    prev_pts = len(pts) + 1
    k = 0
    while prev_pts > n_pts:
        prev_pts = n_pts
        n_pts = len(pts)
        i = -2
        while i < (n_pts - 2):
            Aij = area_of_triangle(centre, pts[i],     pts[(i + 1) % n_pts])
            Ajk = area_of_triangle(centre, pts[(i + 1) % n_pts], \
                                   pts[(i + 2) % n_pts])
            Aik = area_of_triangle(centre, pts[i],     pts[(i + 2) % n_pts])

            if Aij + Ajk < Aik:
                del pts[i+1]
            i += 1
            n_pts = len(pts)
        k += 1
    return n.asarray(pts)

## Catmull-Rom spline stuff
def CatmullRomSpline(P0, P1, P2, P3, nPoints=100):
    """
    P0, P1, P2, and P3 should be 2D numpy arrays that define the Catmull-Rom spline.
    nPoints is the number of points to include in this curve segment.
    """

    # Only calculate points between P1 and P2
    t = n.linspace(1,2,nPoints)

    # Reshape so that we can multiply by the points P0 to P3
    # and get a point for each value of t.
    t = t.reshape(len(t),1)

    A1 = (1-t)*P0 + t*P1
    A2 = (2-t)*P1 + (t-1)*P2
    A3 = (3-t)*P2 + (t-2)*P3

    B1 = (2-t)/2*A1 + t/2*A2
    B2 = (3-t)/2*A2 + (t-1)/2*A3

    C  = (2-t)*B1 + (t-1)*B2
    return C


## My code begins
def pushApart(points, mindist):
    changes = False
    for i in xrange(0,points.shape[0]):
        for j in xrange(i+1,points.shape[0]):

            if (points[i]-points[j]).dot(points[i]-points[j]) < mindist**2:
                # Push apart the points
                h = points[i]-points[j]
                hl = n.sqrt(h.dot(h))

                h /= hl
                dif = mindist - hl
                h *= dif

                points[i] += h/2
                points[j] -= h/2
                changes = True
    return changes


def getopts(argv):
    opts = {}  # Empty dictionary to store key-value pairs.
    while argv:  # While there are arguments left to parse...
        if argv[0][0] == '-':  # Found a "-name value" pair.
            opts[argv[0]] = argv[1]  # Add key and value to the dictionary.
        argv = argv[1:]  # Reduce the argument list by copying it starting from index 1.
    return opts

def newTrack(dimension,difficulty,num_points,filename,maxDisp,mindist,roadWidth):

    splinepoints = 100
    #roadWidth = 6.5

    # Random points in 2D spaced around (0,0)
    p1 = dimension*n.random.rand(2,num_points)-0.5*dimension
    p2 = convex_hull(p1) # points are also sorted in a counter clockwise manner

    # Push too close points apart
    for i in xrange(0,3):
        pushApart(p2,mindist)

    ## Add fun parts
    p3 = n.zeros((2*p2.shape[0],2))

    for i in xrange(0,p2.shape[0]):
        dispLen = random.random()**difficulty * maxDisp
        randAngle = random.random() * 2 * n.pi

        direction = n.array([n.cos(randAngle), n.sin(randAngle)])

        v = direction*dispLen

        p3[2*i] = p2[i]

        # handle last point, connects to the first point
        if i == p2.shape[0]-1:
            p3[2*i+1] = (p2[i] + p2[0])/2.0 + v
        else:
            p3[2*i+1] = (p2[i] + p2[i+1])/2.0 + v

    for i in xrange(0,3):
        pushApart(p3,mindist)

    ## Calculate Catmull-Rom splines
    result = n.zeros((p3.shape[0]*splinepoints,2))
    for i in xrange(0,p3.shape[0]):
        # Handle edge cases
        if i == 0:
            # first
            pp = p3.shape[0]-1
            pn = i+1
            pnn = i+2
        elif i == p3.shape[0]-2:
            # next last point
            pp = i-1
            pn = i+1
            pnn = 0
        elif i == p3.shape[0]-1:
            # last point
            pp = i-1
            pn = 0
            pnn = 1
        else:
            pp = i-1
            pn = i+1
            pnn = i+2

        #print(i)
        P0 = p3[pp]
        P1 = p3[i]
        P2 = p3[pn]
        P3 = p3[pnn]

        C = CatmullRomSpline(P0,P1,P2,P3,splinepoints+1)
        result[i*splinepoints:(i+1)*(splinepoints),:] = C[0:-1,:]

    # Displace the road such that some part of the road is at (0,0)
    # loop through points, find the one closest to the axis
    match = -1
    least = float('inf')

    for i in xrange(0,result.shape[0]):
        if result[i,1] > 0:
            if abs(result[i,0]) < least:
                #print(result[i])
                least = result[i,0]
                match = i

    offset = result[match]
    result -= offset

    # And rotate
    tangent = (result[match+1]-result[match-1])/2
    #print(tangent)
    rot_angle = n.arctan2(tangent[1],tangent[0])
    rot = n.array([[n.cos(rot_angle), -n.sin(rot_angle)],
                    [n.sin(rot_angle), n.cos(rot_angle)]])

    result = n.dot(result,rot)

    ## Print the road specification to one string
    road = '    <road name="ecocar_track">\n'
    road += "        <width>" + str(roadWidth) + "</width>\n"

    # Each point
    for i in xrange(0,result.shape[0]):
        road += "        <point>" + str(result[i,0]) + " " + str(result[i,1]) + " 0</point>\n"

    # Last point (first point again)
    road += "        <point>" + str(result[0,0]) + " " + str(result[0,1]) + " 0</point>\n"
    road += "    </road>\n"

    # Save
    worldpath = os.path.dirname(os.path.dirname(sys.argv[0])) + "/worlds"
    protof = open(worldpath + '/proto.world','r')

    proto = protof.read()

    searchS = "<!-- Insert road here -->"
    line = proto.find(searchS) + len(searchS) + 1

    new_world = proto[:line] + road + proto[line:]

    f = open(worldpath + "/" + filename,'w')
    f.write(new_world)
    f.close()

    shutil.copy(worldpath + "/" + filename, worldpath + "/ecocar.world")

    return worldpath + "/" + filename


## Main code
opts = getopts(sys.argv)

if '-size' in opts:
    dimension = float(opts['-size'])
else:
    dimension = 100.0

if '-diff' in opts:
    difficulty = float(opts['-diff'])
else:
    difficulty = 1

if '-points' in opts:
    num_points = int(opts['-points'])
else:
    num_points = 10

if '-o' in opts:
    filename = opts['-o']
else:
    filename = 'track_' + time.strftime("%Y%m%d-%H%M%S") + '.world'

if '-maxdisp' in opts:
    maxDisp = opts['-maxdisp']
else:
    maxDisp = dimension / 20.0

if '-mindist' in opts:
    mindist = opts['-mindist']
else:
    mindist = dimension / 10.0

if '-roadwidth' in opts:
    roadWidth = opts['-roadwidth']
else:
    roadWidth = 6.5

loadFile = False;
if '-filename' in opts:
    loadFilePath = opts['-filename']
    loadFile = True

if loadFile:
    # Instead of creating a new track, just use the one provided. Copy it into the correct position
    worldpath = os.path.dirname(os.path.dirname(sys.argv[0])) + "/worlds"
    shutil.copy(worldpath + "/" + loadFilePath, worldpath + "/ecocar.world")

else:
    # create new track, print the file name
    newfilename = newTrack(dimension,difficulty,num_points,filename,maxDisp,mindist,roadWidth)
    print(newfilename)
