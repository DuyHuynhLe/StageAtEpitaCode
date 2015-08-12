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
*/

#ifndef frontPropagate_include
#define frontPropagate_include
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

#include <algorithm> //std::sort

namespace mln{
  using namespace std;
  namespace params
  {
    const unsigned score =10;
    const float textSize = 0.5; //ratio of similarity between 2 letters
    const unsigned boxSize = 50; //minimum area of bounding box to be kept (width * height)
    const unsigned area = 30;// perimeter minimum of a component to be kept old =30
    float laplacianThreshold = 50; //thresHold of 25% strongest point of laplacian of a region
	
    const unsigned widthHeightRatio = 10;
    const unsigned heightWidthRatio = 5;
    const unsigned minWidth = 4;
    const unsigned minHeight =6;
	
    //these 2 values are used in the searching for average edge gradient.
    //It search for haft a circle clockwise and then haft a circle counter-clockwise for
    //next possible point.
    const dpoint2d dp[8] = {dpoint2d(-1,1),dpoint2d(0,1), dpoint2d(1,1),dpoint2d(1,0),
			    dpoint2d(1,-1),dpoint2d(0,-1),dpoint2d(-1,-1),dpoint2d(-1,0)};
    //const dpoint2d dpmax[8] = {dpoint2d(1,1),dpoint2d(1,0), dpoint2d(1,-1),dpoint2d(0,-1),
    //		    dpoint2d(-1,-1),dpoint2d(-1,0),dpoint2d(-1,1),dpoint2d(0,1)};
    const unsigned iter[8] = {0,1,1,1,4,7,7,7};
  }
  
  namespace tos{

    //vector<point2d> debug;
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
      vector<unsigned int> parent_array,area; //a table of parent, area of each composant
      vector<point2d> startPoint; //contain all point that belongs to the border and all start point of regions
      vector< vector<point2d> > border,removedBorder,removedLap,removedRatio;
      vector<float> color;// average value of pixel inside a region
      vector<unsigned> contourSize;
      vector<float> lambda;
      vector<float> lap;// average value of pixel inside a region
      vector<float> removedLapTemp; //for debug value of average lap at that region
      vector<boundingBox> boxes;// bounding box of each region except the root
      vector<box2d> boundingBoxes;// the bounding box of posible grouped text 
      unsigned nLabels;
	  
      //constructor
      tos(unsigned reservedSize){
	parent_array.reserve(reservedSize);
	area.reserve(reservedSize);
	startPoint.reserve(reservedSize);
	border.reserve(reservedSize);
	color.reserve(reservedSize);
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
	  if(tree.boxes[i].center[1]+tree.boxes[i].width/2 + tree.boxes[i].height*2> tree.boxes[node].center[1]-tree.boxes[node].width/2)
	    //if(isEquivalent(boxes[i].center[1],boxes[node].center[1],boxes[node].height*2))
	    if(isEquivalent(tree.boxes[i].center[0],tree.boxes[node].center[0],tree.boxes[node].height/2)) //no too different verticalement
	      if(isSimilar(tree.boxes[i].height,tree.boxes[node].height,params::textSize)) //equivalent height
		return true;
	if(tree.boxes[i].center[1]>tree.boxes[node].center[1])
	  if(tree.boxes[i].center[1]-tree.boxes[i].width/2-tree.boxes[i].height*2 < tree.boxes[node].center[1]+tree.boxes[node].width/2)
	    //if(isEquivalent(boxes[i].center[1],boxes[node].center[1],boxes[node].height*2))
	    if(isEquivalent(tree.boxes[i].center[0],tree.boxes[node].center[0],tree.boxes[node].height/2))
	      if(isSimilar(tree.boxes[i].height,tree.boxes[node].height,params::textSize))
		return true;			
	return false;
      }
	  
      vector<unsigned int> scoring(tos& tree){
	/*
	 *Use for smart binarisation. IF a node has similar gray level with its grand parrent,
	 *it contributes 1 point to its Grand parent, if not it contributes 1 point to its parent
	 */	
  
	vector<unsigned int> score = vector<unsigned>(tree.nLabels,0);
	  
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
      mln_piter_(box2d) p(input.domain());
      for_all(p)
	output(p) = 0.2126*input(p).red() + 0.0722*input(p).blue() + 0.7152*input(p).green();
      //output(p) = 0.299*input(p).red() + 0.114*input(p).blue() + 0.587*input(p).green();
      //output(p) =conversion::gray(input(p).red(),input(p).blue(),input(p).green());
      return output;
    }
	
    namespace internal
    {
      template <typename I, typename V>
      float followEdge(const point2d& p,const image2d<unsigned int>& output,const image2d<I>& gradient,const image2d< V >& laplacian,vector<point2d>& bord,
		       unsigned& count,float& lap) // CODE1 last param lap
      {
	/*
	 *Follow the contour of component and return the gradient moyenne
	 *at these points and number of point in the edge
	 */	
	count=0;
	point2d np=p;
	float grad = 0;
	vector < int > lapPoints; //CODE1
	int direction =0;     
	do
	  {//while not return to the original
	    for(int i=0;i <8;i++)
	      {
		direction = (params::iter[i]+direction) %8;//get direction
		//*--------------> todo: replace with special ticket for the border
		if ( output(np + params::dp[direction]) and output(np + params::dp[(direction+1)%8]) == 0
		     and output.domain().has(np + params::dp[(direction+1)%8]))
		  {//if that point is checked and its next point on the left isn't, it is our next point
		    lap += laplacian(np + params::dp[(direction+1)%8]); // CODE0
		    np=np+params::dp[direction];// move to next point
		    bord.push_back(np);//add next point to return vector
		    grad+=gradient(np);//increase gradient
		    count++;//increase counter
			
		    //lapPoints.push_back(laplacian(np)); // for sorted vector of laplacian count % CODE1
		    //lapPoints.push_back(laplacian(np + params::dp[(direction+1)%8]));
		    break;//found, no need further check
		  }
	      }
	  }while(np!=p);

	//if(count>params::area)
	//{
	// ********************************************
	//sort(lapPoints.begin(),lapPoints.end());
	//for(std::vector<int>::iterator j=lapPoints.begin()+count*0.9;j!=lapPoints.end();++j)
	//lap += *j;
	//lap = lap/(count*0.1);
	//********************************************* //CODE1
	return grad/count;
	/*}
	  else
	  {
	  lap = 0;
	  return 0;
	  }*/ //CODE1
      }
      
      template <typename I,typename V, typename K >    
      inline void decision(V gradient, I output,K input, float gradThresHold,point2d p, tos& tree,bool& bounding_box_count_flag,
			   unsigned& level, unsigned& current,short int& x1,short int& x2,short int& y1,short int& y2) //, vector< vector<int> > lapContainer) //CODE2vector<unsigned>& count
      {
	/*
	 *Give the decision if the this zone is a new component or should be merged with
	 *the upper component, initial color (average gray level), area count and bounding box counting
	 *
	 * * For new region,
	 * * We will increase the current level and use it as new ticket, we also put
	 * * the ticket of the top left point of the contour as its parent.
	 * * We also init the area count and color count
	 * * If it is not the border region, the bounding box of last composant will
	 * * be added and new bounding box will be initalized
	 * *
	 * * For region decided to be merge, it will use the ticket of the top left
	 * * point of the contour and the bounding box process will not work (as it
	 * * be included in it parent)
	 */			
	unsigned contourSize;
	float lap;
	if(p != output.domain().pmin() )//not the first region
	  {
	    vector<point2d> bord;
	    //point arrive here must be a new point, at the top left most.
	    float grad =followEdge(p+dpoint2d(0,-1),output,gradient,input,bord,contourSize,lap);
	    if(grad >gradThresHold and contourSize>params::area) //CODE1 and (lap>params::laplacianThreshold or lap<-params::laplacianThreshold) 
	      {
		tree.border.push_back(bord);
		//tree.lap.push_back(lap); //CODE1
		tree.area.push_back(1);// for tree.area and tree.color calculation
		tree.lap.push_back(0); // CODE2 CODE3
		//count.push_back(1); // CODE2
		tree.contourSize.push_back(contourSize); //CODE4
		//for bounding_box;
		tree.boxes.push_back(boundingBox(x1,x2,y1,y2));
		bounding_box_count_flag =true;
		x1=p[0];x2=p[0];y1=p[1];y2=p[1];
		
		current=++level;  //increase counter start of region		
		tree.parent_array.push_back(output(p+dpoint2d(-1,-1))); //mark parenthood
		tree.startPoint.push_back(p); //for debug, mark start points
	      }
	    else
	      {
		current=output(p+dpoint2d(-1,-1)); //merge with upper level
		bounding_box_count_flag =false;
		tree.removedBorder.push_back(bord);
	      }
	  }
	else
	  {
	    tree.color.push_back(0);
	    tree.area.push_back(1);// for tree.area and tree.color calculation
	    tree.lap.push_back(0); // CODE2 CODE3
	    //count.push_back(1); //CODE2
	    tree.contourSize.push_back(0); //CODE4
	    current=++level; // start of region
	  }
      }
    }
    
    template <typename I,typename V >
    image2d<unsigned int> labeling(const image2d<V >& image,const image2d< I >& input,const image2d< V >& gradient,
				   float gradThresHold,tos& tree)
    {
      /*
       *Give the decision if the this zone is a new component or should be merged with
       *the upper component, initial color (average gray level), area count and bounding box counting
       */
	  
      //init of output
      image2d<unsigned int> output(input.domain());
      data::fill(output,0); 
      //init of parenthood vector
      tree.parent_array.push_back(1);
      //init of bounding box counting process
      short int x1=0,x2=0,y1=0,y2=0; bool bounding_box_count_flag;
      //vector<unsigned> count; // CODE2
      vector<float> lapVector; // CODE3
      //to check the sign
      bool sign_flag;
      //processing queue
      p_queue_fast<point2d> Q;
      //tempo point
      point2d q;
      //neighbor iteration
      mln_niter_(neighb2d) n(c4(),q); 
      //ticket management
      unsigned level=0,current;
      //iter (slow)
      mln_piter_(box2d) p(input.domain());
      for_all(p)
      {
	//if this point has been processed in side de queue
	if(output(p))  continue;
	//get the decision
	internal::decision(gradient,output,input,gradThresHold,p,tree,bounding_box_count_flag,
			   level,current,x1,x2, y1,y2);//,lapContainer);  //CODE2 count  
	//mark the first point
	output(p)=current;
	//get the sign of region
	sign_flag = input(p)>0;
	//start of region processing
	Q.push(p); 
      
	while(not Q.empty())
	  {
	    q = Q.pop_front();
	    for_all(n)
	      if( input.domain().has(n) and not output(n) and ( sign_flag == (input(n) > 0) or input(n) == 0 ) ) 
		//if the neighbor is inside the image and it does not belong to any region and its sign is the same or it is null
		{
		  if (bounding_box_count_flag)
		    {
		      x1 = min(x1,n[0]);
		      x2 = max(x2,n[0]);
		      y1 = min(y1,n[1]);
		      y2 = max(y2,n[1]);
		      lapVector.push_back(input(n));//CODE3
		    }
		  //color and area count
		  tree.color[current-1] += image(n);
		  if(input(n)>10 or input(n)<10)
		    {
		      //tree.lap[current-1] +=input(n); //CODE2
		      //++count[current -1 ];
		    }
		  ++tree.area[current-1];
		  //add it to new region
		  output(n)=current;
		  //add its neighbors to queue
		  Q.push(n);
		}
	  }//end of while
	if (bounding_box_count_flag)
	  {//CODE3
	    std::sort(lapVector.begin(),lapVector.end());
	    if(lapVector[tree.area[current-1]/2]>0)
	      for(std::vector<float>::iterator j=lapVector.begin() + tree.area[current-1]*0.90;j!=lapVector.end();++j)
		tree.lap[current-1] += *j;
	    else
	      for(std::vector<float>::iterator j=lapVector.begin() ;j!=lapVector.begin() + tree.area[current-1]*0.10;++j)
		tree.lap[current-1] += *j;			
	    tree.lap[current-1] = tree.lap[current-1]/(tree.area[current-1]*0.10);	  
	    lapVector.clear();
	  }
      }//end of for_all
	  
      tree.boxes.push_back(boundingBox(x1,x2,y1,y2));//add the last bouding box
      
      tree.nLabels = level; // return number of regions
      for(int i =0;i<level;i++) //return the average gray level
	{
	  tree.color[i]=tree.color[i]/tree.area[i];
	  //tree.lap[i]= tree.lap[i]/count[i]; //CODE2
	}
      //Get segmentation error
      vector<float> fidelity(tree.nLabels,0);
      int temp;
      for_all(p)
      {
	temp = image(p) - tree.color[output(p)-1];
	fidelity[output(p)-1] = temp*temp;
      }
      //get Lamda
      for(int i =1;i<level;i++) //return the average gray level
	{
	  tree.lambda.push_back(fidelity[i]/tree.contourSize[i]);
	}
      float max = *std::max_element(tree.lambda.begin(),tree.lambda.end())/255.;
      //cout<<"Max "<<max<<endl;
      if (max<255)
	for(int i =0;i<level-1;i++) //return the average gray level
	  {
	    //std::cout<<tree.lambda[i]<<endl;
	    tree.lambda[i]=tree.lambda[i]/max;
	  }
      return output;
    }

    // code used for smart binarization
    //*-------------------------------------------------------
    //*-------------------------------------------------------
    //*-------------------------------------------------------
    image2d<bool> binarization(const image2d<unsigned int>& labeling,tos& tree,int thresHold)
    {
      vector<unsigned int> score = util::scoring(tree);//a table of score(for binarization) of each composant
      vector<unsigned> binary(score.size(),0);
      //decide which binary value is
      for(int i =0;i<score.size();i++)
	if(score[i]>thresHold and tree.area[i]>=tree.getSubnodeArea(i))
	  //high score and own area higher than sub area
	  binary[i]=1;
	else	
	  if(binary[tree.parent_array[ tree.parent_array[i]-1]-1] and binary[ tree.parent_array[i]-1]==0 )
	    //grand parent is background and parrent is a text --> hole
	    binary[i]=1;

	  
      image2d<bool> output(labeling.domain());
      mln_piter_(box2d) p(labeling.domain());
      //paste binary value into image
      for_all(p)
	output(p) = binary[labeling(p)-1];
      return output;
    }


    // code used for text bounding box using only 1 point in the center
    //*-------------------------------------------------------
    namespace internal_old
    {
      unsigned getNearestNeighbor(const image2d<unsigned>& image,const tos& tree,unsigned i,const dpoint2d& direction)
      {
	point2d p =tree.boxes[i].center;
	for(unsigned j =0;i<tree.boxes[j].width/2;j++)
	  p+=direction;//shift out of the bondary
	unsigned current = image(tree.boxes[i].center);
		
	for(unsigned j = 0; j<3*tree.boxes[i].height;j++)
	  {
	    p+=direction;//shift
	    if(not (image.domain().has(p))) break; //out of domain--> end
	    if(image(p)== current) continue; // still in the same region, moving on
	    if(image(p)!= 1 and (tree.parent_array[i] == tree.parent_array[image(p)-1]) and i!=image(p)-1
	       and util::isSimilar(tree.boxes[i].height,tree.boxes[image(p)-1].height,params::textSize) )
	      return image(p);// found a new ticket which is brother and have similar size
	    else
	      current = image(p); // new ticket but not a brother or having similar size
	  }
	return 0; // found no brother in range
      }    
  
      void neighborSearch(const image2d<unsigned>& image, const tos& tree, vector<unsigned>& left, vector<unsigned>& right)
      {
	left  = vector<unsigned>(tree.nLabels-1,0);
	right = vector<unsigned>(tree.nLabels-1,0);
	for(unsigned i = 1;i<tree.nLabels;i++)
	  {
	    left[i-1]  = getNearestNeighbor(image,tree,i, dpoint2d(0,-1)); 
	    right[i-1] = getNearestNeighbor(image,tree,i, dpoint2d(0,1));
	  }
      }
  
      box2d getBoundingBox(const box2d& B1,const box2d& B2,const box2d& B3)
      {
	return make::box2d(min(min(B1.pmin()[0],B2.pmin()[0]),B3.pmin()[0]),
			   min(min(B1.pmin()[1],B2.pmin()[1]),B3.pmin()[1]),
			   max(max(B1.pmax()[0],B2.pmax()[0]),B3.pmax()[0]),
			   max(max(B1.pmax()[1],B2.pmax()[1]),B3.pmax()[1]));
      }
	  
      bool joinRegionsOld(const tos& tree, vector<bool>& queue,box2d& box,
			  unsigned node,const vector<unsigned>& left, const vector<unsigned>& right,bool direction)
      {
	bool found=false;
	cout<<node<<" "<<(right[node] and node+2 == left[right[node]-2])<<" "<<(left[node] and node+2 == right[left[node]-2])<<" "; 
	if(direction)
	  {
	    if (right[node] and node+2 == left[right[node]-2])
	      {
		box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[right[node]-1].box);
		found =true;
		joinRegionsOld(tree,queue,box,right[node]-2,left,right,true);
	      }
	  }
	else if (left[node] and node+2 == right[left[node]-2])
	  {
	    box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[left[node]-1].box);
	    found = true;
	    joinRegionsOld(tree,queue,box,left[node]-2,left,right,false);	    
	  }
	queue[node]=false;
	return found;
      }
  
      bool joinRegions(const tos& tree, vector<bool>& queue,box2d& box,
		       unsigned node,const vector<unsigned>& left, const vector<unsigned>& right,int oldNode)
      {
	bool found=false;
	if (right[node] and right[node]!=oldNode and node+2 == left[right[node]-2])
	  {
	    box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[right[node]-1].box);
	    found =true;
	    joinRegions(tree,queue,box,right[node]-2,left,right,node+2);
	  }
	  
	if (left[node]and left[node]!=oldNode and node+2 == right[left[node]-2])
	  {
	    box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[left[node]-1].box);
	    found = true;
	    joinRegions(tree,queue,box,left[node]-2,left,right,node+2);	    
	  }
	queue[node]=false;
	return found;
      }  
	 
      vector<box2d> getRegions(const image2d<unsigned>& image,const tos& tree)
      {
	vector<unsigned> left,right;
	vector<bool> queue(tree.nLabels-1,true);
	vector<box2d> boxes;
	int node;bool flag = false;
		
	neighborSearch(image,tree,left,right);
		
	for(int node = tree.nLabels-2;node>=0;node--)		
	  if(queue[node])
	    {
	      box2d box;
	      if(joinRegions(tree,queue,box,node,left,right,0))
		if(box.len(1)*box.len(2)>params::boxSize)
		  boxes.push_back(box);
	    }
  
	return boxes;
      }
    }
    
  
    //new strategy ------------------> more searching line
    //*-------------------------------------------------------
    //*-------------------------------------------------------
    //*-------------------------------------------------------     
    box2d getBoundingBox(const box2d& B1,const box2d& B2,const box2d& B3)
    {
      /*
       *return the bounding box of 3 boxes
       */
      return make::box2d(min(min(B1.pmin()[0],B2.pmin()[0]),B3.pmin()[0]),
			 min(min(B1.pmin()[1],B2.pmin()[1]),B3.pmin()[1]),
			 max(max(B1.pmax()[0],B2.pmax()[0]),B3.pmax()[0]),
			 max(max(B1.pmax()[1],B2.pmax()[1]),B3.pmax()[1]));
    }
    namespace internal
    {
      unsigned getNearestNeighbor(const image2d<unsigned>& image,const tos& tree,unsigned i, point2d p,const dpoint2d& direction)
      {
	/*
	 *return the nearest neighbor of a composant
	 */
	//init current label
	unsigned current = image(p); 
	//shift searching point out of the boundary
	if(direction == dpoint2d(0,-1) )
	  p+=dpoint2d(0,-tree.boxes[i].width/2);
	else
	  p+=dpoint2d(0,tree.boxes[i].width/2);
		  
	for(unsigned j = 0; j<tree.boxes[i].height;j++)
	  {
	    p+=direction;//shift
	    if(not (image.domain().has(p))) break; //out of domain--> end		
	    if(image(p)== current) continue; // still in the same region, moving on		
	    if(image(p)!= 1 and (tree.parent_array[i] == tree.parent_array[image(p)-1]) and i!=image(p)-1
	       and util::isSimilar(tree.boxes[i].height,tree.boxes[image(p)-1].height,params::textSize) )
	      //not the first region, have same parent, have different ticket *fixe?(could remove?) and have
	      //similar size
	      return image(p);// found a new ticket which is brother and have similar size
	    else
	      current = image(p); // new ticket but not a brother or having similar size
	  }
	return 0; // found no brother in range
      }
	  
      struct neighbors
      {
	/*
	 *container for the neighbors
	 */		
	vector<unsigned> left[3];
	vector<unsigned> right[3];
	neighbors(unsigned size)
	{
	  left[0].resize(size);
	  left[1].resize(size);
	  left[2].resize(size);
	  right[0].resize(size);
	  right[1].resize(size);
	  right[2].resize(size);
	};
      };
	  
      inline
      void neighborSearch(const image2d<unsigned>& image,const tos& tree, neighbors& ngh)
      {
	/*
	 *Search for neighbor of all component
	 */
	for(unsigned i = 1;i<tree.nLabels;i++)
	  {
	    dpoint2d dp(tree.boxes[i].height*2/5,0);
	    ngh.left[0][i-1]  = getNearestNeighbor(image,tree,i,tree.boxes[i].center-dp, dpoint2d(0,-1));
	    ngh.left[1][i-1]  = getNearestNeighbor(image,tree,i,tree.boxes[i].center, dpoint2d(0,-1)); 
	    ngh.left[2][i-1]  = getNearestNeighbor(image,tree,i,tree.boxes[i].center+dp, dpoint2d(0,-1));
		  
	    ngh.right[0][i-1] = getNearestNeighbor(image,tree,i,tree.boxes[i].center-dp, dpoint2d(0,1));
	    ngh.right[1][i-1] = getNearestNeighbor(image,tree,i,tree.boxes[i].center, dpoint2d(0,1));
	    ngh.right[2][i-1] = getNearestNeighbor(image,tree,i,tree.boxes[i].center+dp, dpoint2d(0,1));
	  }
      }
	
      inline
      bool doubleValidate(unsigned a,vector<unsigned> neighbor[3],vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros)
      {
	/*
	 * Check if the oposit neighborhood exist
	 */		
	if(node+2 == oNeighbor[0][neighbor[a][node]-2])
	  {bros=neighbor[a][node]; return true;}
	if(node+2 == oNeighbor[1][neighbor[a][node]-2])
	  {bros=neighbor[a][node]; return true;}
	if(node+2 == oNeighbor[2][neighbor[a][node]-2])
	  {bros=neighbor[a][node]; return true;}
	return false;
      }
	  
      inline
      bool subValidate(unsigned a, unsigned b, vector<unsigned> neighbor[3],vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros)
      {
	/*
	 * in case there are 1 label = 0 and 2 label !=0
	 */				
	if(a)
	  if(a==b or b==0)
	    return doubleValidate(a,neighbor,oNeighbor,node, bros);
	if(b)
	  return doubleValidate(b,neighbor,oNeighbor,node, bros);
	return false;
		  
      }
	  
      bool regionsDecisionAll(vector<unsigned> neighbor[3],vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros )
      {
	/*
	 * Only decide to merge 2 neighbors if all evident agreed,
	 */			
	if(neighbor[0][node]*neighbor[1][node]*neighbor[2][node])
	  {
	    if(neighbor[0][node] == neighbor[1][node] and neighbor[0][node] == neighbor[2][node])
	      return doubleValidate(0,neighbor,oNeighbor,node, bros);
	    return false;
	  }
		
	if(neighbor[1][node]*neighbor[2][node]!=0)
	  return subValidate(1,2,neighbor,oNeighbor,node, bros);
	if(neighbor[0][node]*neighbor[2][node]!=0)
	  return subValidate(0,2,neighbor,oNeighbor,node, bros);
	if(neighbor[1][node]*neighbor[0][node]!=0)
	  return subValidate(1,0,neighbor,oNeighbor,node, bros);
		
	return false;
      }
	  
      bool regionsDecision(vector<unsigned> neighbor[3],vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros )
      {
	/*
	 * Decide to merge 2 neighbors using majority of evidents,
	 */			
	if(neighbor[0][node]*neighbor[1][node]*neighbor[2][node])
	  {
	    if(neighbor[0][node] == neighbor[1][node] or neighbor[0][node] == neighbor[2][node])
	      return doubleValidate(0,neighbor,oNeighbor,node, bros);
	    if(neighbor[1][node] == neighbor[2][node])
	      return doubleValidate(1,neighbor,oNeighbor,node, bros);
	    return false;
	  }
		
	if(neighbor[1][node]*neighbor[2][node]!=0)
	  return subValidate(1,2,neighbor,oNeighbor,node, bros);
	if(neighbor[0][node]*neighbor[2][node]!=0)
	  return subValidate(0,2,neighbor,oNeighbor,node, bros);
	if(neighbor[1][node]*neighbor[0][node]!=0)
	  return subValidate(1,0,neighbor,oNeighbor,node, bros);
		
	return false;
      }
	  
      unsigned joinRegionsThree(const tos& tree, vector<bool>& queue,box2d& box,
				unsigned node,neighbors ngh,int oldNode)
      {
	/*
	 * continue searching and joint component on the right and then on the left of a component, 
	 */
	unsigned count=1; 
	bool found=false;
	unsigned bros=0;
	//tag the current node
	queue[node]=false;
	
	if(regionsDecision(ngh.right,ngh.left,node,bros))
	  if(queue[bros-2])
	    {
	      //joint current component with old component
	      box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[bros-1].box);
	      count +=joinRegionsThree(tree,queue,box,bros-2,ngh,node+2);
	    }	
	if(regionsDecision(ngh.left,ngh.right,node,bros))
	  if(queue[bros-2])
	    {
	      //joint current component with old component
	      box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[bros-1].box);
	      count +=joinRegionsThree(tree,queue,box,bros-2,ngh,node+2);	    
	    }
	return count;
      }
	  
      unsigned joinRegionsRange(const tos& tree, vector<bool>& queue,box2d& box,
				unsigned node,int oldNode)
      {
	/*
	 * Effort to make the joint faster. This search using the region array and tree.
	 * and join component using distance of its center.
	 */
	int count =1;
	vector<unsigned> neighbors;
	//Search parent tree
	for(int i = tree.nLabels-1;i>0;i--)
	  {			
	    //check process queue
	    if(queue[i])
	      {
		//check color
		//if(tree.isEquivalent(tree.color[i],tree.color[tree.getParentIndex(i)],params::score))
		//continue;
		//find component having same parent
		if(tree.parent_array[i]!=tree.parent_array[node])
		  continue; 
		//check distance
		if(util::checkCharacter(tree,i,node) and util::checkCharacter(tree,node,i))
		  //check the laplacian
		  //if(tree.isEquivalent(tree.lap[i],tree.lap[node],params::laplacianThreshold*0.5))
		  {
		    //joint box
		    queue[i] = false;
		    box = getBoundingBox(box,tree.boxes[node].box,tree.boxes[i].box);
		    //joint neighbors
		    neighbors.push_back(i);
		  }	
	      }
	  }
	for(int i =0;i<neighbors.size();i++)
	  count+=joinRegionsRange(tree, queue,box,neighbors[i],node);	  
	return count;
      }
    }//end of internal
		
    namespace three{//search neighbors using 3 line
      vector<box2d> getRegions(const image2d<unsigned>& image,const tos& tree)
      {
	//to check if component has been proceed
	vector<bool> queue(tree.nLabels-1,true);
	//tempo box
	vector<box2d> boxes;
	internal::neighbors ngh(tree.nLabels-1);
	internal::neighborSearch(image,tree,ngh);
	//run from leaves to root
	for(int node = tree.nLabels-2;node>=0;node--)		
	  if(queue[node])//if not proceed
	    {
	      box2d box;
	      if(internal::joinRegionsThree(tree,queue,box,node,ngh,0)) //NOTE: this is redundant, if thresholding the box size, no need to check
		if(box.len(1)*box.len(2)>params::boxSize) //if total box size greater than a thresHold
		  //add it
		  boxes.push_back(box);
	    }
	return boxes;
      }
    }
	
    namespace distance{//search for neighbors using distance calculations
      vector<box2d> getRegions(const image2d<unsigned>& image,tos& tree)
      {
	//tempo box
	vector<box2d> boxes;
	//to check if component has been proceed
	vector<bool> queue(tree.nLabels,true);
	/************************************** Average of laplacian, used as a thresHold
		float pAvg=0,nAvg=0;
		unsigned countP=0,countN=0;
		for(std::vector<float>::iterator j=tree.lap.begin();j!=tree.lap.end();++j)
		if (*j>0)
		  {pAvg+=*j;countP+=1;}
		else
		  {nAvg+=*j;countN+=1;}
		pAvg=pAvg/countP;
		nAvg=nAvg/countN;*/
	//remove some node **************************************************
	for(int node = tree.nLabels-1;node>0;node--)
	  {
	    //ocupation and ratio
	    if(tree.boxes[node].height > tree.boxes[node].width*params::widthHeightRatio and tree.boxes[node].width*1./tree.boxes[node].height > params::heightWidthRatio)
	      {queue[node] = false; continue;}
	    if(tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) < 0.1 or (tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) > 0.9 and tree.boxes[node].width > tree.boxes[node].height))
	      {queue[node] = false; tree.removedRatio.push_back(tree.border[node-1]); continue;}			
	    //lap
	    if((tree.lap[node]<params::laplacianThreshold and tree.lap[node]> -params::laplacianThreshold ) or tree.area[node]<30)
	      //if(not tree.isEquivalent(tree.lap[node],-tree.lap[tree.getParentIndex(node)], params::laplacianThreshold)) //CODE5
	      //if(tree.lap[node]<pAvg and tree.lap[node]> nAvg )
	      {queue[node] = false; tree.removedLap.push_back(tree.border[node-1]); tree.removedLapTemp.push_back(tree.lap[node]); continue;}			
	    //height and width
	    if(tree.boxes[node].width<params::minWidth or tree.boxes[node].height <params::minHeight )
	      {queue[node] = false; continue;}

	  }
	//remove some node **************************************************
		
		
	//run from leaves to root
	for(int node = tree.nLabels-1;node>0;node--)		
	  if(queue[node])//if not proceed
	    {
	      box2d box;
	      //clear queue
	      queue[node]=false;
	      //check color
	      //if(not tree.isEquivalent(tree.color[node],tree.color[tree.getParentIndex(node)],params::score))
	      if(internal::joinRegionsRange(tree,queue,box,node,0)>1) //NOTE: this is redundant, if thresholding the box size, no need to check
		if(box.len(1)*box.len(2)>params::boxSize) //if total box size greater than a thresHold
		  //add it
		  boxes.push_back(box);
	    }
	return boxes;
      }
    }
  
    namespace internal{
      template<typename I>
      void max_min(I image,int& min, int& max)
      {
	/*
	 * get the max value of that image
	 */	  
	mln_piter_(box2d) p(image.domain());
	for_all(p)
	  if(image(p)>max) max =image(p);
	  else if (image(p)<min) min = image(p);
      }
    }
	
    template<typename I>
    I normalisation(const I& image)
    {
      mln_piter_(box2d) p(image.domain());
      I output(image.domain());
      int max=0;
      int min=0;
      internal::max_min(image,min,max);
      for_all(p)
      {
	if(image(p)>0) output(p) = 255*image(p)/max;
	else if(image(p)<=0) output(p) = -255*image(p)/min;
	//else  output(p) = 0;
      }
      return output;
    }
  
  }
}

#endif
