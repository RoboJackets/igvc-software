#include "imgUtils.h"
#include <Magick++.h>


using namespace std;
using namespace Magick;
int main(int argc, char**argv)
{
    cout << (*argv);
    InitializeMagick(*argv);
    //string input = "/home/alex/Desktop/release/src/examples/common/example1/input.ppm";
    string rPath = "/home/alex/Desktop/IGVC/2013/software/trainingSets/both/set1/11123906-0.bmp";
    string lPath = "/home/alex/Desktop/IGVC/2013/software/trainingSets/both/set1/11123906-0-Right.bmp";
    //string lPath = "/home/alex/Desktop/GroundTruth/proj2-pair2-L.png";
    //string rPath = "/home/alex/Desktop/GroundTruth/proj2-pair2-R.png";

    Image depthImage;
    getDispInputImg(lPath, rPath, depthImage);
    depthImage.display();
    depthImage.write("/home/alex/Desktop/release/src/examples/common/example1/igvcinput.ppm");
}
