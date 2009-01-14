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

/**
<pre>
  //go through the scenes, find al the matches.
  Matches m = findMatches( *s, *refScene, 1.0, 0.8 );

  // RANSAC
  int bestSupport = 0;
  Matches bestM;
  PCT bestP;

  for( int i=0; i<100; i++ ) {
    //create a PCT from 4 random matches
    PCT P = RANSAC( m );
    //discard bizarre PCTs
    if( P.norm() > 2.0 ) continue;
    // find out how well supported this PCT is
    Matches supportM = getSupport( P, m, 6 );
    // record the best supported PCT.
    if( supportM.size() > bestSupport ) {
        bestSupport = supportM.size();
        bestP = P;
        bestM = supportM;
        //P.print();
    }
  }
  cerr<<" Best support "<<bestSupport<<" : "<<bestM.size() <<" : " ;
</pre>
*/
Matches findMatches( Scene &s0,
                     Scene &s1,
                     float magRadius ,
                     float improvement  ) ;
double distSq( Coords &c0, Coords &c1 ) ;
Matches &findMatchesByPosition( Scene &s0,
                                Scene &s1,
                                float radius,
                                PCT P,
                                int width,
                                int height );

