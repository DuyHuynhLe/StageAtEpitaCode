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
    void label_code(const std::vector<unsigned> parent_vector,const value::rgb8 color[],char * filename)
    {
      /*
       * save the relation node->parent to an image
       */
      int size = parent_vector.size();
      image2d<value::rgb8> output(2,size);
      for(int i=0;i<size;i++)
	{
	  output.at_(0,i) = color[parent_vector[i]-1];
	  output.at_(1,i) = color[i];
	}
      io::ppm::save(output,filename);
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
	else output(p) = value::rgb8(0,0,255);
      }
      io::magick::save(output,filename);

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
    void label_colorization(const I image,char* filename,tos::tos tree,bool bord,bool outBox,bool groupText)
    {
      /*
       * color the labeled image, with option to turn on the border of each element (deleted one included)
       * bounding box of each component and bounding box of grouped component
       */		  
      value::rgb8 color[tree.nLabels];
      unsigned i,j ;
      //create random colors
      for(i =0;i<tree.nLabels;i++)
	{
	  color[i]=value::rgb8( rand() * 255 , rand() * 255 , rand() * 255);
	  //color[i] = value::rgb8(  tree.turn[i]* 255/500 , tree.turn[i]* 255/500 ,  tree.turn[i]* 255/500);
	}
      //paste to the output
      mln_piter_(box2d) p(image.domain());
      image2d<value::rgb8> output(image.domain());
      for_all(p)
      {
	output(p)=color[image(p)-1];
      }
	  
      //center of each box 
      for(int i=0;i<tree.nLabels;i++)
	output(tree.boxes[i].center)=value::rgb8(0,0,0);
      //start point and border
      if(bord)
	{
	  for(i =0;i<tree.nLabels-1;i++)
	    output(tree.startPoint[i])=value::rgb8(0,0,0);
	  for(i=0;i<tree.border.size();i++)
		for(j=0;j<tree.border[i].size();j++)
	    output(tree.border[i][j])=value::rgb8(255,255,255);
		
	  cout<<"removed by grad "<<tree.removedBorder.size()<<endl;
	  for(i=0;i<tree.removedBorder.size();i++)
		for(j=0;j<tree.removedBorder[i].size();j++)
		  output(tree.removedBorder[i][j])=value::rgb8(200,0,0);
		  
	  cout<<"removed by size "<<tree.removed1.size()<<endl;
	  for(i=0;i<tree.removed1.size();i++)
		for(j=0;j<tree.removed1[i].size();j++)
		  output(tree.removed1[i][j])=value::rgb8(0,0,0);		  
		  
	  cout<<"removed by lap "<<tree.removedLap.size()<<endl; 
	  for(i=0;i<tree.removedLap.size();i++)
		for(j=0;j<tree.removedLap[i].size();j++)
		  output(tree.removedLap[i][j])=value::rgb8(0,tree.removedLapTemp[i]>0?tree.removedLapTemp[i]:-tree.removedLapTemp[i]
													,tree.removedLapTemp[i]>0?tree.removedLapTemp[i]:-tree.removedLapTemp[i]);
	  cout<<"removed by ratio "<<tree.removedRatio.size()<<endl; 
	  for(i=0;i<tree.removedRatio.size();i++)
		for(j=0;j<tree.removedRatio[i].size();j++)
		  output(tree.removedRatio[i][j])=value::rgb8(0,0,255);
		  
	  cout<<"removed by contour size "<<tree.removed2.size()<<endl;
	  cout<<"got here?"<<endl;
	  // for(i=0;i<tree.contourSize.size();i++)
		//for(j=0;j<tree.removed2[i].size();j++)
		  //output(tree.removed2[i][j])=value::rgb8(255,255,0);
		  
		  		  
	}
	cout<<"got here:"<<endl;
      //bounding box of each component
      if(outBox)
	for(int i=0;i<tree.boxes.size();i++)
	  mln::draw::box(output,tree.boxes[i].box,value::rgb8(255,0,0));
      //bounding box of grouped component
	  
	cout<<"here:"<<endl;
      if(groupText)
	for(int i=0;i<tree.boundingBoxes.size();i++)
	  mln::draw::box(output,tree.boundingBoxes[i],value::rgb8(0,255,0));
      //for(int i =0;i<tos::debug.size();i++)
      //output(tos::debug[i])=value::rgb8(255,255,255);
	cout<<"or here:"<<endl;  
      label_code(tree.parent_array,color,"label_code.ppm");
      io::magick::save(output,filename);
    }
	
	
    template<typename I>
    void label_colorization(const I image,char* filename,tos::tos tree,bool bord,bool outBox,bool groupText,image2d<value::int_u8> grad)
    {
      /*
       * color the labeled image, with option to turn on the border of each element (deleted one included)
       * bounding box of each component and bounding box of grouped component
       * THIS IS A DUPLICATE TO SHOW THE GRADIENT ON THE BORDER
       */		  
      value::rgb8 color[tree.nLabels];
      unsigned i,j ;
      //create random colors
      for(i =0;i<tree.nLabels;i++)
	{
	  #if blackWhite
		color[i]=value::rgb8(fillColor,fillColor,fillColor);
	  #else
		color[i]=value::rgb8( rand() * 255 , rand() * 255 , rand() * 255);
	  #endif
	}
      //paste to the output
      mln_piter_(box2d) p(image.domain());
      image2d<value::rgb8> output(image.domain());
      for_all(p)
      {
	output(p)=color[image(p)-1];
      }
	  
      //center of each box 
      for(int i=0;i<tree.nLabels;i++)
	output(tree.boxes[i].center)=value::rgb8(0,0,0);
      //start point and border
      if(bord)
	{
	  for(i =0;i<tree.nLabels-1;i++)
	    output(tree.startPoint[i])=value::rgb8(0,0,0);
	}
	#if gradView
	  for(i=0;i<tree.border.size();i++)
	  for(j=0;j<tree.border[i].size();j++)
	    output(tree.border[i][j])=value::rgb8(grad(tree.border[i][j]),grad(tree.border[i][j]),grad(tree.border[i][j]));
	  for(i=0;i<tree.removedBorder.size();i++)
	  for(j=0;j<tree.removedBorder[i].size();j++)
	    output(tree.removedBorder[i][j])=value::rgb8(grad(tree.removedBorder[i][j]),grad(tree.removedBorder[i][j]),grad(tree.removedBorder[i][j]));
    #else
	  for(i=0;i<tree.border.size();i++)
	  for(j=0;j<tree.border[i].size();j++)
	    output(tree.border[i][j])=value::rgb8(tree.lambda[i],0,0);
	#endif
      //bounding box of each component
      if(outBox)
	for(int i=0;i<tree.boxes.size();i++)
	  mln::draw::box(output,tree.boxes[i].box,value::rgb8(255,0,0));
      //bounding box of grouped component
      if(groupText)
	for(int i=0;i<tree.boundingBoxes.size();i++)
	  mln::draw::box(output,tree.boundingBoxes[i],value::rgb8(0,255,0));
      //for(int i =0;i<tos::debug.size();i++)
      //output(tos::debug[i])=value::rgb8(255,255,255);
	  
      //label_code(tree.parent_array,color,"label_code.ppm");
      io::magick::save(output,filename);
    }	
    
    void box_on_image(image2d<value::rgb8> image,tos::tos tree, char* filename)
    {
      /*
       * color the labeled image, with option to turn on the border of each element (deleted one included)
       * bounding box of each component and bounding box of grouped component
       */	
      box2d current;
      for(int i=0;i<tree.boundingBoxes.size();i++)
	{
	  current = make::box2d(tree.boundingBoxes[i].pmin()[0],tree.boundingBoxes[i].pmin()[1],tree.boundingBoxes[i].pmax()[0],tree.boundingBoxes[i].pmax()[1]);
	  mln::draw::box (image,current,value::rgb8(255,0,0));
	}
      io::magick::save(image,filename);
    }
	
    void saveGT(tos::tos tree,char* filename)
    {
	  ofstream outFile (filename);
	  if (outFile.is_open())
	  {
		for(int i = 0;i<tree.boundingBoxes.size();i++)
		  outFile<<tree.boundingBoxes[i].pmin()[1]<<","<<tree.boundingBoxes[i].pmin()[0]<<","<<tree.boundingBoxes[i].pmax()[1]<<","<<tree.boundingBoxes[i].pmax()[0]<<"\r\n";
		outFile.close();
	  }
	}
	
  }
}

#endif
