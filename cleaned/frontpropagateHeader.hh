//todo: tranformer from vector to arrray
/*
Note: checking the laplacian of a region. It should be strong to be keep.
test average of only the strongest point

* Tested: Laplacian on contour approach 1: CODE0
*Tested: Laplacian thresHolding: on contour: not really impressive CODE1.
*Tested: Laplacian on whole region CODE2 different thresHold
*tested: conversion d'image --> seem good.
*Tested: only strong point in images // CODE3
*Tested: coefficient : in display change grad view to 0
*Teting : avg lap distance CODE 5: Problem: not always true, sometime  strong region is included in another strong region. FAIL
*Done: Replace copy with reference: reduce time
*Doing: separate tos to tos, binary, textbox
**Testing max of Laplacian //CODE6
* Turning //CODE7
*/

#ifndef frontPropagateHeader_include
#define frontPropagateHeader_include
#include <mln/core/image/image2d.hh>
#include <mln/core/site_set/p_queue_fast.hh>
#include <mln/core/alias/neighb2d.hh>
#include <mln/value/int_u8.hh>
#include <mln/util/array.hh>
#include <mln/core/alias/box2d.hh>
#include <vector>
#include <mln/value/rgb8.hh>
#include <mln/debug/println.hh>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <sstream>
#include <string>
#include "parametre.hh"
#include <algorithm> //std::sort

namespace mln{
  namespace tos{

    struct boundingBox{ //contain the information for the Bounding Box of each element and 
      box2d  box;
      point2d center;
      unsigned height;
      unsigned width;
      boundingBox(short int x1,short int x2, short int y1, short int y2)
      {box = make::box2d(x1,y1,x2,y2); height = x2-x1+1; width = y2-y1+1; center = box.pcenter(); }
    };
    
    struct tos
    {
      std::vector<unsigned int> parent_array,area,text; //a table of parent, area of each composant
      std::vector<point2d> startPoint; //contain all point that belongs to the border and all start point of regions
      std::vector< std::vector<point2d> > border,removedBorder,removedLap,removedRatio,removedTurn, removed1,removed2,removed3,removed4;
      std::vector<float> color;// average value of pixel inside a region
      std::vector<unsigned> contourSize;
      std::vector<float> lambda;
      std::vector<float> lap;// average value of pixel inside a region
      std::vector<float> removedLapTemp; //for debug value of average lap at that region
      std::vector<boundingBox> boxes;// bounding box of each region except the root
      std::vector<box2d> boundingBoxes;// the bounding box of posible grouped text
	  std::vector<float> turn;
      unsigned nLabels;
	  
      //constructor
      tos(unsigned reservedSize){
    text.reserve(reservedSize/2);
    color.reserve(reservedSize);
	parent_array.reserve(reservedSize);
	area.reserve(reservedSize);
	startPoint.reserve(reservedSize);
	border.reserve(reservedSize);
	boxes.reserve(reservedSize);
	boundingBoxes.reserve(reservedSize/2);
	lap.reserve(reservedSize);
      }
	  
      //methode
      unsigned int getParentIndex(unsigned int i) {return parent_array[i]-1;} // we start ticket from 1, 0 is for did not processed
      unsigned int getGrandParentIndex(unsigned int i) {return getParentIndex(getParentIndex(i));} //value in parentArray is parent ticket
      unsigned getSubnodeArea(unsigned index){
	/*
	 *Use for smart binarisation. get a sum of all subnode area
	 */	

	unsigned output=0;
	for(int i =0;i<nLabels;i++)
	  if(index and parent_array[i]==index+1)
	    output+= area[i];
	return output;
      }	      
    };

    namespace util{
      template <typename N>  
      bool isEquivalent(N number1,N number2,unsigned thresHold){
	/*
	 *Euclide distance between 2 point, compare with a thresHold
	 */
	return(number1-number2)*(number1-number2) <thresHold*thresHold;
      }	  
      template <typename N>  
      bool isSimilar(N number1,N number2,float thresHold){
	/*
	 *similarity ratio between 2 number min(n1,n2)/max(n1,n2), compare with a thresHold 
	 */	
  
	return number1<number2?(number1/float(number2)>thresHold):(number2/float(number1)>thresHold);
      }
      bool checkCharacter(const tos& tree,int i, int node)
      {
	/*
	 *Check if 2 point are close enought
	 */
	if(tree.boxes[i].center[1]<tree.boxes[node].center[1])
	  if(int(tree.boxes[i].center[1]+tree.boxes[i].width/2 + tree.boxes[i].height*params::SeCof) > int(tree.boxes[node].center[1]-tree.boxes[node].width/2))
	    //if(isEquivalent(boxes[i].center[1],boxes[node].center[1],boxes[node].height*2))
	    if(isEquivalent(tree.boxes[i].center[0],tree.boxes[node].center[0],tree.boxes[node].height/2)) //no too different verticalement
	      if(isSimilar(tree.boxes[i].height,tree.boxes[node].height,params::textSize)) //equivalent height
		return true;
	if(tree.boxes[i].center[1]>tree.boxes[node].center[1])
	  if(int(tree.boxes[i].center[1]-tree.boxes[i].width/2-tree.boxes[i].height*params::SeCof) < int(tree.boxes[node].center[1]+tree.boxes[node].width/2))
	    //if(isEquivalent(boxes[i].center[1],boxes[node].center[1],boxes[node].height*2))
	    if(isEquivalent(tree.boxes[i].center[0],tree.boxes[node].center[0],tree.boxes[node].height/2))
	      if(isSimilar(tree.boxes[i].height,tree.boxes[node].height,params::textSize))
			return true;
	return false;
      }
	  
      std::vector<unsigned int> scoring(tos& tree){
	/*
	 *Use for smart binarisation. IF a node has similar gray level with its grand parrent,
	 *it contributes 1 point to its Grand parent, if not it contributes 1 point to its parent
	 */	
  
	std::vector<unsigned int> score = std::vector<unsigned>(tree.nLabels,0);
	  
	for(int i =0;i<tree.nLabels;i++)
	  {
	    if(isEquivalent(tree.color[i],tree.color[tree.getGrandParentIndex(i)],params::score))
	      score[tree.getGrandParentIndex(i)] +=1;
	    else
	      score[tree.getParentIndex(i)] +=1;
	  }
	return score;
      }	  
    }
    namespace conversion{
      const double rY = 0.212655;
      const double gY = 0.715158;
      const double bY = 0.072187;
	  
      // Inverse of sRGB "gamma" function. (approx 2.2)
      double inv_gam_sRGB(value::int_u8 ic) {
	double c = ic/255.0;
	if ( c <= 0.04045 )
	  return c/12.92;
	else 
	  return pow(((c+0.055)/(1.055)),2.4);
      }
	  
      // sRGB "gamma" function (approx 2.2)
      value::int_u8 gam_sRGB(double v) {
	if(v<=0.0031308)
	  v *= 12.92;
	else 
	  v = 1.055*pow(v,1.0/2.4)-0.055;
	return value::int_u8(v*255+.5);
      }
	  
      // GRAY VALUE ("brightness")
      value::int_u8 gray(value::int_u8 r, value::int_u8 g, value::int_u8 b) {
	return gam_sRGB(
			rY*inv_gam_sRGB(r) +
			gY*inv_gam_sRGB(g) +
			bY*inv_gam_sRGB(b)
			);
      }
    }
	
    image2d<value::int_u8> transformBnW(const image2d<value::rgb8> input)
    {
      image2d<value::int_u8> output(input.domain());
      initialize(output, input);//same structure
      border::fill(output, 300); // so that borders stop the queue-based!
      const unsigned N = input.nelements(); // pixels, including those of the border
      
      for (unsigned p = 0; p < N; ++p) // p is now an "unsigned"
        if(output.element(p) != 300)
          output.element(p) = 0.2126*input.element(p).red() + 0.0722*input.element(p).blue() + 0.7152*input.element(p).green();
      return output;
    }
	
	  
  }//TOS namespace
}//mln namespace

#endif
