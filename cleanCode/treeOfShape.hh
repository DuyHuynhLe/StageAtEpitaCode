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

        namespace tosUtil{
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
	  
    }
    

    struct boundingBox
    { //contain the information for the Bounding Box of each element and 
      box2d  box;
      point2d center;
      unsigned height;
      unsigned width;
      boundingBox(short int x1,short int x2, short int y1, short int y2)
      {box = make::box2d(x1,y1,x2,y2); height = x2-x1+1; width = y2-y1+1; center = box.pcenter(); };
      boundingBox(void){};
    };

    struct node
    {
      //relationship
      node *parent;
      std::vector<node*> children;
      unsigned label;
      //properties
      unsigned area;
      float grad;
      std::vector<point2d> border;
      unsigned contourSize;
      boundingBox box;// bounding box of each region except the root
      bool isText,isReal;
      unsigned distance;
      
      //constructor
      node(unsigned a, unsigned cz, float g, std::vector<point2d> b ,boundingBox bx,unsigned l)
      {
	area = a; border = b; contourSize = cz; box = bx; label = l;isText=false;isReal=true;grad = g;
      };
      
      node(void)
      {isText = false;isReal=true;};
      //methods
      void setParent(node *p)
      {parent = p;};

      void addChildren(node *c)
      {children.push_back(c);};
      void calculateDistance(void)
      {
	distance = 0;
	if(children.size())
	  {
	    unsigned minDistance = children[0]->distance+1;
	    for(unsigned i = 1;i<children.size();i++)
	      minDistance = minDistance<children[i]->distance?minDistance:children[i]->distance+1;
	    distance =minDistance;
	  }
      }
      
    };// end of node struct
    
    
    struct tos
    {
      std::vector< std::vector<point2d> >removedBorder,removedLap,removedRatio,removedTurn, removed1,removed2,removed3,removed4;
      std::vector<box2d> boundingBoxes;// the bounding box of posible grouped text
      std::vector<unsigned> parent_array,text;
      std::vector<node> nodes;
      unsigned nLabels; 
      tos(unsigned reservedSize)
      {
	parent_array.reserve(reservedSize/2);
	nodes.reserve(reservedSize/2);
	nodes.push_back(node());
	text.reserve(reservedSize/4);
	boundingBoxes.reserve(reservedSize/4);
	nLabels = 0;
	
      };
      //methods

      //add a new node
      void addNode(unsigned a, unsigned cz,float g,std::vector<point2d> b , boundingBox bx, unsigned parent, unsigned label)
      {
	nodes.push_back(node(a,cz,g,b,bx,label));
	nodes[label].setParent(&nodes[parent]);
	nodes[parent].addChildren(&nodes[label]);
	nLabels+=1;
      }
      //remove a node
      void removeNode(unsigned node)
      {
	
	for(unsigned i = 0;i<nodes[node].children.size();i++)
	  {
	    //update parent's info
	    //nodes[node].parent->area += nodes[node].area;
	    //update parent's child
	    //add new
	    nodes[node].parent->children.push_back(nodes[node].children[i]);
	    //update child's parent
	    nodes[node].children[i]->parent = nodes[node].parent;
	  }
	//remove current node
	for(unsigned j = 0;j<nodes[node].parent->children.size();j++)
	  if(nodes[node].parent->children[j]->label == node)
	    nodes[node].parent->children.erase(nodes[node].parent->children.begin()+j);
	//remove
	nodes[node].isReal=false;
	nodes[node].isText=false;
	
      }
      //get subnodes area
      unsigned getSubnodeArea(unsigned index){
	/*
	 *Use for smart binarisation. get a sum of all subnode area
	 */	

        unsigned output=0;
        for(unsigned i =1;i<=nLabels;i++)
          if(index and parent_array[i]==index)
            output+= nodes[i].area;
        return output;
      }

      void calculateDistanceFromLeaves (void)
      {
	for(unsigned node =nLabels; node>0;node--)
	    nodes[node].calculateDistance();
      }

      void pruneTree(void)
      {
	for(unsigned node = nLabels;node>1;node--)
	  {
	    if(nodes[node].area*1./(nodes[node].box.width*nodes[node].box.height) < 0.1)// 
	      {	removeNode(node) ; removedRatio.push_back(nodes[node].border); continue; }
	    
	    if(nodes[node].area<30)
	      {removeNode(node) ; removedLap.push_back(nodes[node].border);  continue;}
	    
	    if(nodes[node].contourSize*1./(nodes[node].box.width + nodes[node].box.height)>8 )
	      {removeNode(node) ; continue;}   
	  }
      }

      bool isClose(unsigned node1, unsigned node2)
      {
	if(int(nodes[node1].box.center[1]-nodes[node1].box.width/2)<int(nodes[node2].box.center[1]+nodes[node2].box.width/2+nodes[node2].box.height*params::SeCof)
	   or
	   int(nodes[node1].box.center[1]+nodes[node1].box.width/2)<int(nodes[node2].box.center[1]-nodes[node2].box.width/2-nodes[node2].box.height*params::SeCof)
	   )
	  if(int(nodes[node2].box.center[1]-nodes[node2].box.width/2)<int(nodes[node1].box.center[1]+nodes[node1].box.width/2+nodes[node1].box.height*params::SeCof)
	     or
	     int(nodes[node2].box.center[1]+nodes[node2].box.width/2)<int(nodes[node1].box.center[1]-nodes[node1].box.width/2-nodes[node1].box.height*params::SeCof)
	     )
	  // check distance

	    if(tosUtil::isEquivalent(nodes[node1].box.center[0],nodes[node2].box.center[0],nodes[node1].box.height/2)) //check distance vertical
	      if(tosUtil::isEquivalent(nodes[node2].box.center[0],nodes[node1].box.center[0],nodes[node2].box.height/2)) //check distance vertical
		if(tosUtil::isSimilar(nodes[node1].box.height,nodes[node2].box.height,params::textSize))//check height
		  return true;
	return false;
      }
      
    }; //end of tos Struct


    image2d<value::int_u8> transformBnW(const image2d<value::rgb8> input)
    {
      std::cout<<" got here \n";
	
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
