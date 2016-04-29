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
#include "treeOfShape.hh"

namespace mln{
  using namespace std;

  
  namespace tos{	
    box2d getBoundingBox(const box2d& B1,const box2d& B2)
    {
      /*
       *return the bounding box of 3 boxes
       */
      return make::box2d(min(B1.pmin()[0],B2.pmin()[0]),
			 min(B1.pmin()[1],B2.pmin()[1]),
			 max(B1.pmax()[0],B2.pmax()[0]),
			 max(B1.pmax()[1],B2.pmax()[1])
			 );
    }
    namespace internal
    {
  
      unsigned joinRegionsRange(tos& tree, std::vector<bool>& queue,box2d& box,
				unsigned node,std::vector<unsigned>& text)
      {
	/*
	 * Effort to make the joint faster. This search using the region array and tree.
	 * and join component using distance of its center.
	 */
	int count = 1;
	std::vector<unsigned> neighbors;
	//Search parent tree
	unsigned parent = tree.nodes[node].parent->label;
	//find component having same parent
	for(unsigned i = 0;i<tree.nodes[parent].children.size();i++)
	  {
	    unsigned sibling = tree.nodes[parent].children[i]->label;
	    //check process queue
	    if(queue[sibling])
	      {	
		//check distance
		if(tree.isClose(sibling,node))
		  {
		    //joint box
		    queue[sibling] = false;
		    box = getBoundingBox(box,tree.nodes[sibling].box.box);
		    //joint neighbors
		    neighbors.push_back(sibling);
		    text.push_back(sibling);
		  }	
	      }
	  }
	for(unsigned i =0;i<neighbors.size();i++)
	  count+=joinRegionsRange(tree, queue,box,neighbors[i],text);
	
	return count;
      }
    }//end of internal
		
    namespace distance{//search for neighbors using distance calculations
      std::vector<box2d> getRegions(const image2d<unsigned>& image,tos& tree)
      {
	//tempo box
	std::vector<box2d> boxes;
	//to check if component has been proceed
	std::vector<bool> queue(tree.nLabels+1);	

	tree.pruneTree();
	
	tree.calculateDistanceFromLeaves();
	
	for(unsigned i = 1;i<=tree.nLabels;i++)
	  queue[i]=tree.nodes[i].isReal;
	for(unsigned i = 1;i<=tree.nLabels;i++)
	  if(tree.nodes[i].distance>=1)
	    queue[i]=false;


		
	//run from leaves to root
	for(int node = tree.nLabels;node>1;node--)		
	  if(queue[node])//if not proceed
	    {
	      box2d box=tree.nodes[node].box.box;
	      //clear queue
	      queue[node]=false;
	      std::vector<unsigned> text;
	      text.push_back(node);
	      //check color
	      //if(not tree.isEquivalent(tree.color[node],tree.color[tree.getParentIndex(node)],params::score))
	      if(internal::joinRegionsRange(tree,queue,box,node,text)>1)
		{
		  boxes.push_back(box);
		  for(unsigned i =0;i<text.size();i++)
		    {tree.nodes[text[i]].isText = true;}
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
