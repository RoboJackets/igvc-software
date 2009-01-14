/**
 * This file is part of the OpenVIDIA project at http://openvidia.sf.net
 * Copyright (C) 2004, James Fung
 *
 * OpenVIDIA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * OpenVIDIA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenVIDIA; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 **/
#ifndef __FV_UTIL
#define __FV_UTIL
#ifdef WIN32
#include "C:\Documents and Settings\FORDPREFECT\Desktop\simple_downsample\libopenvidia\include\openvidia\openvidia32.h"
#else
#include <openvidia/openvidia32.h>
#endif

///Returns the euclidean 2d distance between this feature point's location
/// and the given coordinates
double distSq( Coords &c0, Coords &c1 ) {
    double X,Y;
    X = c0.x() - c1.x();
    Y = c0.y() - c1.y();
    return X*X + Y*Y ;
}


inline float featureDiffSq( Feature &f0, Feature &f1 )
{
// assert( f0.descriptor.size() == 128 );
// assert( f1.descriptor.size() == 128 );
    double ssd = 0.0;
    for ( int i=0 ; i<128 ; i++ ) {
        //  double diff = (f0.descriptor[i]-f1.descriptor[i]);
        double diff = (f0.descArray[i] - f1.descArray[i]);
        ssd += (double)diff*(double)diff;
    }
    return (float)ssd;
}

Feature *findBestMatch( Feature &f0, Scene &s1, float magRadius, float thresh)
{
    float dist1 = 9999;
    float dist2 = 9999;

//  vector<Feature>::iterator it1
    Feature *bestMatch ;

    //for( it1 = s1.features.begin() ; it1 != s1.features.end() ; it1++ ) {
    for (int i = 0 ; i<s1.features.size() ; i++ ) {

        // only check those vectors who have nearby magnitudes (restricts it to
        //  a shell of radius it->magnitude around the origin )

        if ( fabsf(f0.magnitude - s1.features[i].magnitude) < magRadius ) {

            float dist = featureDiffSq ( f0, s1.features[i] );
            if ( dist < dist1 ) {
                dist2 = dist1 ;
                dist1 = dist ;
                bestMatch = &(s1.features[i]);
            }
            else if ( dist < dist2 ) {
                dist2 = dist;
            }//endif

        }//endiff


    }//endfor

    // returns if the match is well defined (not a noisy clump)
    if ( dist1 < thresh*thresh * dist2  ) {
        //return &(*bestMatch) ;
        return bestMatch;
    }
    else {
        return NULL;
    }
}

Matches findMatches( Scene &s0,
                     Scene &s1,
                     float magRadius ,
                     float improvement  )
{
    Matches matches  ;
    vector<Feature>::iterator it0 ;
    //cerr<<"Comparing "<<s0.features.size()<<" to "<<s1.features.size()<<endl;
    //cerr<<"start\n";
    for ( it0 = s0.features.begin() ; it0 != s0.features.end() ; it0++ ) {
        Feature *bestMatch = NULL;
        if ( ( bestMatch = findBestMatch( *it0, s1, magRadius, improvement ) ) != NULL ) {
            Match match = Match( &(*it0), bestMatch  );
            matches.push_back( match );
        }
    }
    //cerr<<"Done "<<matches.size() <<" matches found"<<endl;
    return matches;
}

Feature *findBestMatchByPosition( Feature &f0, Scene &s1,
                                  float radiusSq, PCT &P, int width, int height)
{
    vector<Feature>::iterator it;
    Coords n;
    !P;

    for ( it = s1.features.begin() ; it != s1.features.end() ; it++ )
    {
        if ( distSq( (n = P.project( *it, width, height )), f0) < radiusSq )
        {
            return &(*it);
        }
    }
    return NULL;
}


Matches &findMatchesByPosition( Scene &s0,
                                Scene &s1,
                                float radius,
                                PCT P,
                                int width,
                                int height )
{
    Matches *matches = new Matches() ;
    vector<Feature>::iterator it0;
    for ( it0 = s0.features.begin() ; it0 != s0.features.end() ; it0++ )
    {
        Feature *bestMatch = NULL;
        if ( ( bestMatch = findBestMatchByPosition( *it0, s1, radius, P, width, height) ) != NULL )
        {
            Match *match = new Match( &(*it0), bestMatch  );
            matches->push_back( *match );
        }
    }
    cerr<<"Done "<<matches->size() <<" matches found"<<endl;
    return *matches;
}

#endif
