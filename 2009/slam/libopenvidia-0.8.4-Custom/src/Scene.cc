
#ifdef WIN32
#include "C:\Documents and Settings\FORDPREFECT\Desktop\simple_downsample\libopenvidia\include\openvidia\openvidia32.h"
#define METHOD_EXACT 0
#define METHOD_SVD   1

#else
//#include "Scene.h"
#include <openvidia/openvidia32.h>
#endif
Scene::Scene(int x ) {
    //features = vector<Feature>(x);
    //features.reserve(x*sizeof(Feature));
}


void Scene::saveToDisk(const char *filename) {
    ofstream saveFile;
    saveFile.open(filename, ios::trunc );
    saveFile << features.size() << endl;

    if ( features.size()  == 0 ) cerr<<"EMPTY FILE "<<endl;
    for ( vector<Feature>::iterator it = features.begin();
            it != features.end();
            it++ ) {
        saveFile << it->x() <<" "<< it->y()<< " " ;
        saveFile << it->orientation << " ";
        saveFile << it->dx <<" "<< it->dy << " " << it->magnitude << " ";

        for ( int i=0 ;i<128; i++ ) {
            saveFile << it->descArray[i] <<" ";
        }
        saveFile << endl;
    }
    saveFile.close();
    return;
}

/// Default constructor creates empty scene (empty vector of features )
Scene::Scene()
{
}

/// Constructor is given a filename, which is loaded.
Scene::Scene(const char *filename) {
    ifstream saveFile;
    saveFile.open(filename);
    if ( !saveFile.good() ) {
        cerr<<" Could not open save file! No data loaded. "<<endl;
        return ;
    }
    int numFeat;
    saveFile >> numFeat;
    //cerr << "NumFeat read : " << numFeat << endl;

    // for each of the features...
    for ( int i=0 ; i<numFeat ; i++ ) {
        //create a new feature
        Feature *f = new Feature;
        int x, y;
        saveFile >> x ;
        saveFile >> y ;
        f->set( x,  y);
        saveFile >> f->orientation ;
        saveFile >> f->dx ;
        saveFile >> f->dy ;
        saveFile >> f->magnitude ;

        for ( int j=0  ; j<128 ; j++ ) {
            // push back values onto descriptor
            float val;
            saveFile >> val;
            f->descArray[j] = val ;
        }
        //cerr<<"Feature loaded, magnitude : "<<f->magnitude<<endl;
        //oh my stars. we've lost orientation data - but we never really used that
        // for matching anyways..
        this->features.push_back( *f);
        delete(f);
    }
    saveFile.close();
}

PCT Scene::RANSAC( Matches &m, int width, int height )
{
    //create a Matches object to hold our selected correspondences.
    Matches ransacCorrs ;
    if ( m.size() < 4 ) {
        return PCT() ;
    }
    //choose 4 correspondences to define a PCT.
    while ( ransacCorrs.size() < 4 ) {

        //create a random index into the matches vector to choos a random match
        int idx = (int)(floorf((float)rand()/((float)RAND_MAX)*(float)m.size() ));

        if ( idx >= m.size() ) idx = m.size()-1;
        //make sure we dont re-use the same data point.
        //todo: if a collision, then use a circular linear search instead of reprobe
        if ( find( ransacCorrs.begin(),
                   ransacCorrs.end(), m[idx] ) == ransacCorrs.end() ) {
            ransacCorrs.push_back(m[idx]);
        }

    }
    PCT P( ransacCorrs, height, width, METHOD_EXACT );
    /*
      //print out corr2p
      Matches::iterator it;
      it = ransacCorrs.begin();
      cerr<<endl<<height<<" "<<width<<" ";
      while( it != ransacCorrs.end() ) {
        cerr<<it->first->y()<<" "<<it->first->x()<<" ";
        cerr<<it->second->y()<<" "<<it->second->x()<<" ";
        it++;
      }
      cerr<<endl;
      P.print();
      cerr<<endl;
      ~P;
      P.print();
      cerr<<endl;
    */
    ///ransacCorrs.clear();
    return P;
}


///given an array of matches, m, how many support the hypothesized
/// PCT, P.

Matches Scene::getSupport( PCT &P, Matches &m, float radius, int width, int height )
{
    Matches supportedMatches ;
    float radiusSq = radius*radius;
    Coords n;
    Matches::iterator it;

    it = m.begin();
    while ( it != m.end() ) {


        if ( distSq( (n = P.project( *(it->first), width, height )), *(it->second) ) < radiusSq )
        {
            /*
                   cerr<<"Comparing "<< it->first->x() <<", "<< it->first->y() ;
                   cerr<<" to "<< it->second->x() << ", " << it->second->y() << endl;
                   n = P.project( *(it->first), width, height ) ;
                   cerr<<"With "; P.print();
                   cerr<<endl<< it->first->x() <<", "<< it->first->y()<<" projects to ";
                   cerr<<n.x()<<" "<<n.y()<<endl;
                   cerr<<"distance of "<<distSq( (n = P.project( *(it->first), width, height )), *(it->second) )<<endl<<endl;
            */

            supportedMatches.push_back( *it);
        }
        it++;
    }
//cerr<<"supported matches size "<<supportedMatches.size()<<endl;
    return supportedMatches ;
}

/// Determine how well two scene match each other.

Matches Scene::MatchTo( Scene *s, int w, int h, PCT &bestP )
{
    Matches bestM;

    //go through the scenes, find al the matches.
    Matches m = findMatches( *s, *this, 1.0, 0.8 );

    // RANSAC
    unsigned int bestSupport = 0;

    for ( int i=0; i<200; i++ ) {
        //create a PCT from 4 random matches
        PCT P = RANSAC( m,w,h );

        //discard bizarre PCTs
        if ( P.norm() > 2.0 ) {
            continue;
        }
        // find out how well supported this PCT is
        Matches supportM = getSupport( P, m, 6 ,w,h);
        // record the best supported PCT.
        if ( supportM.size() > bestSupport ) {
            bestSupport = supportM.size();
            bestP = P;
            bestM = supportM;
        }
    }
    //cerr<<"bestM.size() "<<bestM.size();

    return bestM;
}

///This function trims out the given matches OUT of the scene.
int Scene::TrimOutMatches ( Matches *m )
{
    /*
      Matches::iterator it;
      int numTrimmed = 0;

      // s->features is the vector of features to be subtracted.
      // features is the local features to be sutracted from.

      // go thru all the matches, find the corresponding feature.
      for( it = m->begin(); it != m->end() ; it++ )
      {
        cerr<<"searching: "<<it->first->x()<<","<<it->first->y()<<" -> ";
        cerr<<it->second->x()<<","<<it->second->y()<<endl;
        //check all the features
        for( vector<Feature>::iterator f = features.begin(); f != features.end() ; f++ )
        {
            // if the feature which generated the match is found...
            if( ( it->second->x() == f->x() ) &&
                ( it->second->y() == f->y() ) )
            {
                //remove it.
                cerr<<"Trimmed out feature at :"<<f->x()<<","<<f->y()<<endl;
                features.erase(f);
                numTrimmed++;
                break;
            }
        }

      }
      // we should have trimme all the matches out.
      //assert(numTrimmed == m->size() );
      cerr<<" trimmed "<<numTrimmed<<" of "<<m->size()<<endl;
      return numTrimmed;
    */
    return 0;
}

void Scene::AddFeaturesToScene( Scene *SceneToAdd, Matches *m, PCT P, int width, int height )
{
    //add all the new features from the new scene.
    // but don't ad those that are already matched.
    int c = 0;

    for ( vector<Feature>::iterator featureToAdd = SceneToAdd->features.begin();
            featureToAdd != SceneToAdd->features.end();
            featureToAdd++ )
    {
        // first go thru the matches, ignore it if its matched.
        Matches::iterator it;
        for ( it = m->begin(); it != m->end(); it++ )
        {
            if ( it->second == &(*featureToAdd) )
            {
                c++;
                break;
            }
        }
        // if we reached the end of the matches array, then no match was found

        if ( it == m->end() ) {

            Feature *nfeat = new Feature();
            *nfeat = *featureToAdd;

            // update coordinates to put into scene reference frame coordinates.
            Coords nCoords = P.project( *featureToAdd, width, height ) ;

            nfeat->first = nCoords.x();
            nfeat->second = nCoords.y();

            features.push_back( *nfeat );

            cerr<<"Added feature at :"<<featureToAdd->x() << "," <<featureToAdd->y() <<" into reference coordinate at ";
            cerr<<nfeat->x()<<","<<nfeat->y()<<endl;

        }

    }
    cerr<<" Found "<<c<<" of "<<m->size();
    cerr<<"Total : "<<features.size()<<endl;

}
