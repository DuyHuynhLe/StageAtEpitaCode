#ifndef param_include
#define param_include
#include <vector>
    
  namespace mln{
  
    namespace params
    {
      unsigned score =10;
      float textSize = 0.5; //ratio of similarity between 2 letters
      unsigned boxSize = 25; //minimum area of bounding box to be kept (width * height)
      unsigned area = 7;// perimeter minimum of a component to be kept old =30
      float laplacianThreshold = 0; //thresHold of 25% strongest point of laplacian of a region
      
      unsigned widthHeightRatio = 10;
      unsigned heightWidthRatio = 5;
      unsigned minWidth = 1;
      unsigned minHeight =1; //Minimum to form a E
      const unsigned SeCof = 2;
      const unsigned SearchingCoefficient = 2;
      
      //these 2 values are used in the searching for average edge gradient.
      //It search for haft a circle clockwise and then haft a circle counter-clockwise for
      //next possible point.
      const dpoint2d dp[8] = {dpoint2d(-1,1),dpoint2d(0,1), dpoint2d(1,1),dpoint2d(1,0),
                  dpoint2d(1,-1),dpoint2d(0,-1),dpoint2d(-1,-1),dpoint2d(-1,0)};
      //const dpoint2d dpmax[8] = {dpoint2d(1,1),dpoint2d(1,0), dpoint2d(1,-1),dpoint2d(0,-1),
      //		    dpoint2d(-1,-1),dpoint2d(-1,0),dpoint2d(-1,1),dpoint2d(0,1)};
      const unsigned iter[8] = {0,1,1,1,4,7,7,7};
    }
    
 }
 
#endif