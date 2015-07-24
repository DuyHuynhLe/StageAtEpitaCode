/*
 *Big change: normalisation the laplacian before continue
 *
 */

#include <iostream>
#include <mln/core/image/image2d.hh>

#include <mln/debug/println.hh>
#include "frontpropagate.hh"
#include <mln/io/pgm/load.hh>

#include <tos_src/immerse.hh>

#include <mln/win/rectangle2d.hh>
#include <mln/morpho/laplacian.hh>
#include <mln/morpho/gradient.hh>
#include <mln/arith/times.hh>

#include <vector>

#include <mln/data/fill.hh> //for debugging
#include <mln/io/pgm/save.hh>
#include <mln/io/pbm/save.hh>
#include <mln/io/magick/load.hh>
#include <mln/io/magick/save.hh>
#include <mln/value/rgb8.hh>
#include <mln/fun/v2v/rgb_to_int_u.hh>
#include "display.hh"
#include <mln/linear/gaussian.hh>

#define color 0

void help(char * argv[])
{
  
  std::cerr << "usage: " << argv[0] << " input.ext Laplace_window gradian_window laplaceOutput.ext labelOutput.ext parenthoodOutput.ext gradian_threadholding.ext contour(1|0) elementBox(1|0) textBox(1|0) textBox_ori(1|0) textBox_ori.ext " << std::endl;
  std::abort();

}

int main(int argc, char* argv[])
{
  
  if (argc < 8)
    help(argv);
  
  using namespace std;
  using namespace mln;
  //input
  image2d<value::rgb8> inputrgb;
  io::magick::load(inputrgb,argv[1]);
  image2d<value::int_u8> input = tos::transformBnW(inputrgb);

  //blur out the image
  //if(atof(argv[13]))
    //input = linear::gaussian(input, atof(argv[13]));
  //io::magick::save(input,argv[14]);
  io::magick::save(tos::immerse2(input),argv[14]);
  //*---> Laplacian 
  //create structure element
  win::rectangle2d lapw( atoi(argv[2]),atoi(argv[2]));
  win::rectangle2d gradw( atoi(argv[3]), atoi(argv[3]));
  image2d<int> lap(input.domain());

  //Calculate Laplacian
  morpho::laplacian(input, lapw, lap);
  //Calculate gradient
  /*************************************
   *
   */
  image2d<value::int_u8> input_blur = linear::gaussian(input, atof(argv[13]));
  //***********************************
  //image2d<value::int_u8> grad = morpho::gradient(input,gradw);
  image2d<value::int_u8> grad = morpho::gradient(input_blur,gradw);
  
  
  arith::times_cst(lap,2,lap);//2*laplacian to make sure lap is pair
  //arith::times_cst(grad,2,grad);//2*laplacian to make sure lap is pair
  
  lap = tos::immerse2(lap);
  lap = tos::normalisation(lap); // BIGCHANGE
  //save the laplacian
  display::laplacian_colorization(lap,argv[4]);

  //immerse the gradient
  grad = tos::immerse2(grad);
  //save the gradient
  //io::pgm::save(grad,"grad.pgm");  
  //init output value
  tos::tos treeOfShape(lap.nrows()*lap.ncols());

  arith::times_cst(input,2,input);//2*laplacian to make sure input is pair this input will be used for average gray level calculation 
  params::laplacianThreshold=atoi(argv[6]);
  image2d<unsigned int> output_4 = tos::labeling(tos::immerse2(input),lap,grad,30,treeOfShape);//replace 30 = atof(argv[6]) (gradThresHold)
  
  //get an output
  std::cout<<"number of Labels: "<<treeOfShape.nLabels<<endl;
  //get the bounding box
  if(atoi(argv[10]))
    {
      treeOfShape.boundingBoxes = tos::distance::getRegions(output_4,treeOfShape);
    }
  //get the bounding box and display on original image
  if(atoi(argv[10]) and atoi(argv[11]) )
    {
      display::box_on_image(inputrgb,treeOfShape,argv[12]);
    }
  //displace the labels with different color with color labels
  #if color
  display::label_colorization(output_4,argv[5],treeOfShape,atoi(argv[8]),atoi(argv[9]),atoi(argv[10]));
  #else
  display::label_colorization(output_4,argv[5],treeOfShape,atoi(argv[8]),atoi(argv[9]),atoi(argv[10]),grad);
  #endif
  display::saveGT(treeOfShape,argv[15]);
        
  //io::pbm::save(tos::binarization(output_4,treeOfShape,3),argv[7]);
  //for(int i =0;i<treeOfShape.lambda.size();i++)
  //  cout<<treeOfShape.lambda[i]<<" ";
  //  cout<<endl;
  
  //cout<<treeOfShape.lambda.size()<<" "<<treeOfShape.nLabels<<" "<<treeOfShape.border.size()<<endl;
}
//attention, the tree code lable from 1, but the vector starts from 0