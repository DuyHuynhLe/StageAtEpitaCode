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

#ifndef frontPropagateGrouping_include
#define frontPropagateGrouping_include
#include "frontpropagateHeader.hh"

namespace mln{
  using namespace std;

  
  namespace tos{	
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
	std::vector<unsigned> left[3];
	std::vector<unsigned> right[3];
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
      bool doubleValidate(unsigned a,std::vector<unsigned> neighbor[3],std::vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros)
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
      bool subValidate(unsigned a, unsigned b, std::vector<unsigned> neighbor[3],std::vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros)
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
	  
      bool regionsDecisionAll(std::vector<unsigned> neighbor[3],std::vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros )
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
	  
      bool regionsDecision(std::vector<unsigned> neighbor[3], std::vector<unsigned> oNeighbor[3],unsigned node, unsigned& bros )
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
	  
      unsigned joinRegionsThree(const tos& tree, std::vector<bool>& queue,box2d& box,
				unsigned node,neighbors ngh,int oldNode,std::vector<unsigned>& text)
      {
	/*
	 * continue searching and joint component on the right and then on the left of a component, 
	 */
	unsigned count=1; 
	bool found=false;
	unsigned bros=0;
	//tag the current node
	queue[node]=false;
	text.push_back(node+1);
	if(regionsDecision(ngh.right,ngh.left,node,bros))
	  if(queue[bros-2])
	    {
	      //joint current component with old component
	      box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[bros-1].box);
	      count +=joinRegionsThree(tree,queue,box,bros-2,ngh,node+2,text);
	    }	
	if(regionsDecision(ngh.left,ngh.right,node,bros))
	  if(queue[bros-2])
	    {
	      //joint current component with old component
	      box = getBoundingBox(box,tree.boxes[node+1].box,tree.boxes[bros-1].box);
	      count +=joinRegionsThree(tree,queue,box,bros-2,ngh,node+2,text);	    
	    }
	return count;
      }
	  
      unsigned joinRegionsRange(const tos& tree, std::vector<bool>& queue,box2d& box,
				unsigned node,int oldNode,std::vector<unsigned>& text)
      {
	/*
	 * Effort to make the joint faster. This search using the region array and tree.
	 * and join component using distance of its center.
	 */
	int count = 1;
	std::vector<unsigned> neighbors;
	//Search parent tree
	for(int i = tree.nLabels-1;i>=0;i--)
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
            text.push_back(i+1);
		  }	
	      }
	  }
	for(int i =0;i<neighbors.size();i++)
	  count+=joinRegionsRange(tree, queue,box,neighbors[i],node,text);
	
	return count;
      }
    }//end of internal
		
    namespace three{//search neighbors using 3 line
      std::vector<box2d> getRegions(const image2d<unsigned>& image,tos& tree)
      {
	//to check if component has been proceed
	std::vector<bool> queue(tree.nLabels,true);
    
    for(int node = tree.nLabels-1;node>0;node--)
	  {
	    //ocupation and ratio
	    //if(tree.boxes[node].height > tree.boxes[node].width*params::widthHeightRatio and tree.boxes[node].width*1./tree.boxes[node].height > params::heightWidthRatio)
	      //{queue[node] = false; continue;}
	    if(tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) < 0.1)// or (tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) > 0.9 and tree.boxes[node].width > tree.boxes[node].height))
	      {queue[node-1] = false; tree.removedRatio.push_back(tree.border[node-1]); continue;}			
	    //lap
	    if(tree.area[node]<30)
	      //if(not tree.isEquivalent(tree.lap[node],-tree.lap[tree.getParentIndex(node)], params::laplacianThreshold)) //CODE5
	      //if(tree.lap[node]<pAvg and tree.lap[node]> nAvg )
	      {queue[node-1] = false; tree.removedLap.push_back(tree.border[node-1]); tree.removedLapTemp.push_back(tree.lap[node]); continue;}			
	    //height and width
	    //if(tree.boxes[node].width<params::minWidth or tree.boxes[node].height <params::minHeight )
	      //{queue[node] = false; continue;}
	    //if(tree.contourSize[node]*1./tree.area[node] < 0.1)
	      //{queue[node] = false; tree.removedRatio.push_back(tree.border[node-1]); continue;}				  

	  }
	//remove some node **************************************************
	//tempo box
	std::vector<box2d> boxes;
	internal::neighbors ngh(tree.nLabels-1);
	internal::neighborSearch(image,tree,ngh);
	//run from leaves to root
	for(int node = tree.nLabels-2;node>=0;node--)		
	  if(queue[node])//if not proceed
	    {
	      box2d box;
          std::vector<unsigned> text;
          text.push_back(node+1);
	      if(internal::joinRegionsThree(tree,queue,box,node,ngh,0,text)>1) //NOTE: this is redundant, if thresholding the box size, no need to check
		//if(box.len(1)*box.len(2)>params::boxSize) //if total box size greater than a thresHold
		  //add it
          {
		  boxes.push_back(box);
          //std::cout<<"text size = "<< text.size()<<std::endl;
          tree.text.insert(tree.text.end(), text.begin(), text.end());
          }
	    }
	return boxes;
      }
    }
	
    namespace distance{//search for neighbors using distance calculations
      std::vector<box2d> getRegions(const image2d<unsigned>& image,tos& tree)
      {
	//tempo box
	std::vector<box2d> boxes;
	//to check if component has been proceed
	std::vector<bool> queue(tree.nLabels,true);
	for(int node = tree.nLabels;node>0;node--)
	  {
	    //ocupation and ratio
	    //if(tree.boxes[node].height > tree.boxes[node].width*params::widthHeightRatio and tree.boxes[node].width*1./tree.boxes[node].height > params::heightWidthRatio)
	      //{queue[node] = false; continue;}
	    if(tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) < 0.1)// or (tree.area[node]*1./(tree.boxes[node].width*tree.boxes[node].height) > 0.9 and tree.boxes[node].width > tree.boxes[node].height))
	      {queue[node] = false; tree.removedRatio.push_back(tree.border[node-1]); continue;}			
	    //lap
	    if((tree.lap[node]<params::laplacianThreshold and tree.lap[node]> -params::laplacianThreshold ) or tree.area[node]<30)
	      //if(not tree.isEquivalent(tree.lap[node],-tree.lap[tree.getParentIndex(node)], params::laplacianThreshold)) //CODE5
	      //if(tree.lap[node]<pAvg and tree.lap[node]> nAvg )
	      {queue[node] = false; tree.removedLap.push_back(tree.border[node-1]); tree.removedLapTemp.push_back(tree.lap[node]); continue;}			
	    //height and width
	    //if(tree.boxes[node].width<params::minWidth or tree.boxes[node].height <params::minHeight )
	      //{queue[node] = false; continue;}
	    //if(tree.contourSize[node]*1./tree.area[node] < 0.1)
	      //{queue[node] = false; tree.removedRatio.push_back(tree.border[node-1]); continue;}				  

	  }
	//remove some node **************************************************
		
		
	//run from leaves to root
	for(int node = tree.nLabels;node>0;node--)		
	  if(queue[node])//if not proceed
	    {
	      box2d box;
	      //clear queue
	      queue[node]=false;
          std::vector<unsigned> text;
	      //check color
	      //if(not tree.isEquivalent(tree.color[node],tree.color[tree.getParentIndex(node)],params::score))

	      if(internal::joinRegionsRange(tree,queue,box,node,0,text)>1) //NOTE: this is redundant, if thresholding the box size, no need to check
		//if(box.len(1)*box.len(2)>params::boxSize) //if total box size greater than a thresHold
		  //add it
          {
            boxes.push_back(box);
            std::cout<<"text size = "<< text.size()<<std::endl;
            tree.text.insert(tree.text.end(),text.begin(),text.end());
          }

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
