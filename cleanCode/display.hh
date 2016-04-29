#ifndef colorisation_include
#define colorisation_include
#include <mln/core/image/image2d.hh>
#include <mln/value/rgb8.hh>
#include <mln/io/ppm/save.hh>
#include <stdlib.h>
#include "frontpropagate.hh"
#include <mln/io/magick/load.hh>
#include <mln/io/magick/save.hh>
#include <mln/core/alias/box2d.hh>
#include <fstream>
#define blackWhite 0
#define fillColor 255
#define gradView 1

namespace mln{
  namespace display{
    template<typename I>
    int max_abs(I image)
    {
      /*
       * get the max value of that image
       */	  
      int max = 0;
      mln_piter_(box2d) p(image.domain());
      for_all(p)
	if(image(p)>max) max =image(p);
	else if (-image(p)>max) max = -image(p);
      return max;
    }    

    template<typename I>
    void laplacian_colorization(const I image, char* filename)
    {
      /*
       * color the laplacian red = negatif, blue = true zero, green = positif
       */	  
      mln_piter_(box2d) p(image.domain());
      image2d<value::rgb8> output(image.domain());
      for_all(p)
      {
	if(image(p)<0) output(p) = value::rgb8(value::int_u8(-image(p)),0,0);
	else if(image(p)>0) output(p) = value::rgb8(0,value::int_u8(image(p)),0);
	else output(p) = value::rgb8(0,0,0);
      }
      io::magick::save(output,filename);

    }
    
    template<typename I>
    void laplacian_BlackNWhite(const I image, char* filename)
    {
      /*
       * color the laplacian red = negatif, blue = true zero, green = positif
       */	  
      mln_piter_(box2d) p(image.domain());
      int max = max_abs(image);
      image2d<value::int_u8> output(image.domain());
      for_all(p)
      {
	if(image(p)<0) output(p) =128+value::int_u8(image(p))*128/max;
	else if(image(p)>0) output(p) = 128+ value::int_u8(image(p))*127/max;
	else output(p) = 128;
      }
      io::magick::save(output,filename);

    }
    
    void saveData(tos::tos tree,value::rgb8 color[])
    {
      ofstream outFile ("data.txt");
      if (outFile.is_open())
	{
	  for(unsigned i = 1;i<=tree.nLabels;i++)
	    if(tree.nodes[i].isReal)
	      outFile<<color[i]<<";"<<tree.nodes[i].grad<<";"<<tree.nodes[i].box.height<<";"<<tree.nodes[i].box.width<<";"<<tree.nodes[i].contourSize<<";"<<tree.nodes[i].distance<<";"<<tree.nodes[i].area<<"\n";
	  outFile.close();
	  //One pixels border
	}
    }
    
	
    template<typename I>
    void laplacian_colorizationNotDependValue(const I image, char* filename)
    {
      /*
       * color the laplacian red = negatif, blue = true zero, green = positif
       */	  
      mln_piter_(box2d) p(image.domain());
      image2d<value::rgb8> output(image.domain());
      for_all(p)
      {
	if(image(p)<0) output(p) = value::rgb8(255,0,0);
	else if(image(p)>0) output(p) = value::rgb8(0,255,0);
	else output(p) = value::rgb8(0,0,255);
      }
      io::magick::save(output,filename);

    }	
	
    template<typename I>
    void label_colorization(const I image,char* filename,char* filename2,tos::tos tree,bool bord,bool outBox,bool groupText)
    {
      /*
       * color the labeled image, with option to turn on the border of each element (deleted one included)
       * bounding box of each component and bounding box of grouped component
       */		  
      value::rgb8 color[tree.nLabels];
      value::rgb8 color2[tree.nLabels];
      unsigned i,j;
      //create random colors
      for(i =1;i<=tree.nLabels;i++)
	{
	  color[i]=value::rgb8( rand() * 255 , rand() * 255 , rand() * 255);
	}
      // for(unsigned node =1;node<=tree.nLabels;node++)
      // 	if(tree.nodes[node].isReal)
      // 	  {
      // 	    if(tree.nodes[node].isText)
      // 	      {
      // 	      color[node]= value::rgb8( rand() * 255 , rand() * 255 , rand() * 255 );
      // 	      color2[node]=color[node];
      // 	      }
      // 	    else
      // 	      {
      // 		color[node]= value::rgb8( rand() * 125 , rand() * 125 , rand() * 125);
      // 		olor2[node]=value::rgb8(255 ,255 ,255);
      // 	      }
      // 	  }
      // 	else
      // 	  {
      // 	    color2[node] = color2[tree.nodes[node].parent->label];
      // 	    color[node] =  color[tree.nodes[node].parent->label];
      // 	  }
      //create black and white image
      for(unsigned node =1;node<=tree.nLabels;node++)
	if(tree.nodes[node].isReal)
	  {
	    if(tree.nodes[node].isText)
	      color2[node]=color[node];
	    else
	      color2[node]=value::rgb8(255 ,255 ,255);
	  }
	else
	  {
	    color2[node] = color2[tree.nodes[node].parent->label];
	    color[node] =  color[tree.nodes[node].parent->label];
	  }

      //paste to the output
      mln_piter_(box2d) p(image.domain());
      image2d<value::rgb8> output(image.domain().len(0)-2,image.domain().len(1)-2);
      image2d<value::rgb8> output2(image.domain().len(0)-2,image.domain().len(1)-2);
      point2d n;

      for_all(p)
      {
	n =p+dpoint2d(-1,-1);
	if (output.domain().has(n) )
	  {
	    output(n)=color[image(p)];
	    output2(n)=color2[image(p)];
	  }
      }

      //start point and border
      if(bord)
	{  
	  for(unsigned node=1;node<tree.nodes.size();node++)
	    for(i=0;i<tree.nodes[node].border.size();i++)
	      if(tree.nodes[node].isReal)
		output(tree.nodes[node].border[i]+dpoint2d(-1,-1))=value::rgb8(255,255,255);
		
	  cout<<"removed by grad "<<tree.removedBorder.size()<<endl;
	  for(i=0;i<tree.removedBorder.size();i++)
	    for(j=0;j<tree.removedBorder[i].size();j++)
	      output(tree.removedBorder[i][j]+dpoint2d(-1,-1))=value::rgb8(200,0,0);
		  
	  cout<<"removed by size "<<tree.removed1.size()<<endl;
	  for(i=0;i<tree.removed1.size();i++)
	    for(j=0;j<tree.removed1[i].size();j++)
	      output(tree.removed1[i][j]+dpoint2d(-1,-1))=value::rgb8(0,0,0);		  
		  
	  cout<<"removed by ratio "<<tree.removedRatio.size()<<endl; 
	  for(i=0;i<tree.removedRatio.size();i++)
	    for(j=0;j<tree.removedRatio[i].size();j++)
	      output(tree.removedRatio[i][j]+dpoint2d(-1,-1))=value::rgb8(0,0,255);
		  
	  cout<<"removed by contour size "<<tree.removed2.size()<<endl;
	  for(i=0;i<tree.removed2.size();i++)
	    for(j=0;j<tree.removed2[i].size();j++)
	      output(tree.removed2[i][j]+dpoint2d(-1,-1))=value::rgb8(255,255,0);
		  
		  		  
	}

      //bounding box of each component
      if(outBox)
	for(unsigned i=1;i<tree.nodes.size();i++)
	  {
	    if(tree.nodes[i].isReal)
	      mln::draw::box(output2,tree.nodes[i].box.box,value::rgb8(0,0,0));
	  } 

      if(groupText)
	for(unsigned i=0;i<tree.boundingBoxes.size();i++)
	  {
	    box2d current = make::box2d(tree.boundingBoxes[i].pmin()[0]-1,tree.boundingBoxes[i].pmin()[1]-1,tree.boundingBoxes[i].pmax()[0]-1,tree.boundingBoxes[i].pmax()[1]-1);
	    mln::draw::box(output2,current,value::rgb8(0,0,0));
	  }

      //test
      saveData(tree,color);
      
      io::magick::save(output,filename);
      io::magick::save(output2,filename2);
    }
	
    
    void box_on_image(image2d<value::rgb8> image,tos::tos tree, char* filename)
    {
      /*
       * color the labeled image, with option to turn on the border of each element (deleted one included)
       * bounding box of each component and bounding box of grouped component
       */	
      box2d current;
      for(unsigned i=0;i<tree.boundingBoxes.size();i++)
	{
	  current = make::box2d(tree.boundingBoxes[i].pmin()[0]-1,tree.boundingBoxes[i].pmin()[1]-1,tree.boundingBoxes[i].pmax()[0]-1,tree.boundingBoxes[i].pmax()[1]-1);
	  mln::draw::box (image,current,value::rgb8(255,0,0));
	}
      io::magick::save(image,filename);
    }
	
    void saveGT(tos::tos tree,char* filename)
    {
      ofstream outFile (filename);
      if (outFile.is_open())
	{
	  for(unsigned i = 0;i<tree.boundingBoxes.size();i++)
	    outFile<<tree.boundingBoxes[i].pmin()[1]<<","<<tree.boundingBoxes[i].pmin()[0]<<","<<tree.boundingBoxes[i].pmax()[1]<<","<<tree.boundingBoxes[i].pmax()[0]<<"\r\n";
	  outFile.close();
	  //One pixels border
	}
    }
    void saveGT_format2(tos::tos tree,char* filename)
    {
      ofstream outFile (filename);
      if (outFile.is_open())
	{
	  for(unsigned i = 0;i<tree.boundingBoxes.size();i++)
	    outFile<<tree.boundingBoxes[i].pmin()[1]<<","<<tree.boundingBoxes[i].pmin()[0]<<","<<tree.boundingBoxes[i].pmax()[1]<<","<<tree.boundingBoxes[i].pmin()[0]<<","<<tree.boundingBoxes[i].pmax()[1]<<","<<tree.boundingBoxes[i].pmax()[0]<<","<<tree.boundingBoxes[i].pmin()[1]<<","<<tree.boundingBoxes[i].pmax()[0]<<"\r\n";
	  outFile.close();
	  //One pixels border
	}
    }

	
  }
}

#endif
