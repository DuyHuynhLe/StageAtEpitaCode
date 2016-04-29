#ifndef frontPropagate_include
#define frontPropagate_include
#include "treeOfShape.hh"

namespace mln{
  using namespace std;

  
  namespace tos{	
    namespace internal
    {
      template <typename I, typename V>
      float followEdge(const point2d& p,const image2d<I>& gradient,const image2d< V >& laplacian, image2d<value::int_u8>& marker ,std::vector<point2d>& bord,
		       unsigned& count, boundingBox& box2d) // CODE1 last param lap //CODE7 turn ,float& turn
      {	 
	/*
	 *Follow the contour of component and return the gradient moyenne
	 *at these points and number of point in the edge
	 */

	count=0;
	//turn = 0; CODE7
	point2d np=p;
	float grad = 0;
	std::vector < int > lapPoints; //CODE1
	short xmin = p[0] ,xmax = p[0],ymin = p[1],ymax = p[1];
	mln_niter_(neighb2d) n(c4(),np);
	p_queue_fast<point2d> Q;
	Q.push(p);
	marker(p)=2;
	while(not Q.empty())
	  {
	    np = Q.pop_front();
		  
	    //if that point is checked and its next point on the left isn't, it is our next point
	    bord.push_back(np);//add next point to return vector
	    grad+=gradient(np);//increase gradient
	    count++;//increase counter
	    xmin = xmin>np[0]? np[0]: xmin;
	    xmax = xmax>np[0]? xmax : np[0];
	    ymin = ymin>np[1]? np[1]: ymin;
	    ymax = ymax>np[1]? ymax : np[1];
	    for_all(n)
	      if(marker.domain().has(n) and marker(n)==1)
		{
		  Q.push(n);
		  marker(n)=2;
		}
		  
	  }
	   
	box2d = boundingBox(xmin,xmax,ymin,ymax);
		
	return grad/count;
      }
	  
      
      template <typename I,typename V, typename K >    
      inline void decision(V gradient, I output,K input,image2d<value::int_u8>& marker, float gradThresHold,point2d p, tos& tree,
			   unsigned& level, unsigned& currentLevel)//short int& x1,short int& x2,short int& y1,short int& y2) //, vector< vector<int> > lapContainer) //CODE2vector<unsigned>& count
      {
	/*
	 *Give the decision if the this zone is a new component or should be merged with
	 *the upper component, initial color (average gray level), area count and bounding box counting
	 *
	 * * For new region,
	 * * We will increase the currentLevel level and use it as new ticket, we also put
	 * * the ticket of the top left point of the contour as its parent.
	 * * We also init the area count and color count
	 * * If it is not the border region, the bounding box of last composant will
	 * * be added and new bounding box will be initalized
	 * *
	 * * For region decided to be merge, it will use the ticket of the top left
	 * * point of the contour and the bounding box process will not work (as it
	 * * be included in it parent)
	 */
	boundingBox box = boundingBox(0,0,0,0);
	unsigned contourSize;
	if(level )//not the first region
	  {
	    std::vector<point2d> bord;
	    //point arrive here must be a new point, at the top left most.
	    float grad =followEdge(p,gradient,input,marker,bord,contourSize,box); // CODE7 ,turn);
	    //std::cout<<"grad = "<<grad<<" box.width= "<<box.width<<endl;
	    if(grad >gradThresHold and contourSize>params::area
	       and box.width > params::minWidth
	       and box.height > params::minHeight
	       and box.height < box.width*params::widthHeightRatio
	       and box.width < box.height*params::heightWidthRatio
	       ) //CODE1 and (lap>params::laplacianThreshold or lap<-params::laplacianThreshold)  CODE7 //and turn < 200
	      {
		currentLevel=++level;  //increase counter start of region
		tree.addNode(1,contourSize,grad,bord,box,output(p+dpoint2d(-1,0)),currentLevel); //new node
	      }
	    else
	      {
		currentLevel=output(p+dpoint2d(0,-1)); //merge with upper level

		if(box.width <= params::minWidth or box.height<= params::minHeight)
		  tree.removed1.push_back(bord);
		else if(contourSize<=params::area)
		  tree.removed2.push_back(bord);
		else if(box.width >= box.height*params::heightWidthRatio or box.height >= box.width*params::widthHeightRatio)
		  tree.removedRatio.push_back(bord);
		else
		  tree.removedBorder.push_back(bord);        
	      }
	  }
	else
	  {
	    currentLevel=++level; // start of region
	    std::vector<point2d> bord;
	    tree.addNode(1,contourSize,30,bord,box,currentLevel,currentLevel); //new node
	  }
      }
    }
		
    template <typename I,typename V >
    image2d<unsigned int> labelling(const image2d<V >& image,const image2d< I >& input,const image2d< V >& gradient,
				    float gradThresHold,tos& tree)
    {
      /*
       *Give the decision if the this zone is a new component or should be merged with
       *the upper component, initial color (average gray level), area count and bounding box counting
       */
	  
      //init of output
      image2d<unsigned int> output(input.domain());
      image2d<value::int_u8> marker(input.domain());
      data::fill(output,0);
      data::fill(marker,0);
      //init of parenthood vector
      //tree.parent_array.push_back(1);
      //to check the sign
      bool sign_flag;
      //processing queue
      p_queue_fast<point2d> Q;
      //tempo point
      point2d q;
      mln_niter_(neighb2d) n;
      mln_niter_(neighb2d) n1(c4(),q);
      mln_niter_(neighb2d) n2(c8(),q);
      //ticket management
      unsigned maxLevel=0,currentLevel;
      //iter (slow)
      mln_piter_(box2d) p(input.domain());
      for_all(p)
      {
	//if this point has been processed in side de queue
	if(output(p))  continue;
	//get the decision
	internal::decision(gradient,output,input,marker,gradThresHold,p,tree,
			   maxLevel,currentLevel);//,x1,x2, y1,y2);//,lapContainer);  //CODE2 count  
	//mark the first point
	output(p)=currentLevel;
	//neighbor iteration
	mln_niter_(neighb2d) n(c4(),q);
	//get the sign of region
	sign_flag = input(p)>0;
	//start of region processing
	Q.push(p); 
	//propagate
	while(not Q.empty())
	  {
	    q = Q.pop_front();
	    if(marker(q))
	      {
		n=n1;
	      }
	    else
	      {
		n=n2;	  
	      }    
	    //neighbor iteration
	    for_all(n)
	      if( input.domain().has(n) )
		{
		  if(not output(n))
		    {

		      if ( sign_flag == (input(n) > 0) or input(n) == 0 ) 
			//if the neighbor is inside the image and it does not belong to any region and its sign is the same or it is null
			{
			  //color and area count
			  ++tree.nodes[currentLevel].area;
			  //add it to new region
			  output(n)=currentLevel;
			  //add its neighbors to queue
			  Q.push(n);
			}
		      else
			{
			  marker(n) = 1;
			}
		    }
		}
	  
	  }//end of while
	  
      }//end of for_all  
      return output;
    }//end of labelling
  
  }
}

#endif
