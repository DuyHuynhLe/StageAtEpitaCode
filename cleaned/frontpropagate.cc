/*
 *Big change: normalisation the laplacian before continue
 *Big change: Laplacian is float
 */

/*
 * CODE1 Displace the laplacian of different value
 *
 *
 */

#include <iostream>
#include <mln/core/image/image2d.hh>

#include <mln/debug/println.hh>
#include <mln/io/pgm/load.hh>

#include <tos_src/immerse.hh>

#include <mln/win/rectangle2d.hh>
#include <mln/morpho/laplacian.hh>
#include <mln/morpho/gradient.hh>
#include <mln/arith/times.hh>

#include <vector>

#include <mln/data/fill.hh> //for debugging
#include <mln/util/timer.hh> //for timer
#include <mln/io/pgm/save.hh>
#include <mln/io/pbm/save.hh>
#include <mln/io/magick/load.hh>
#include <mln/io/magick/save.hh>
#include <mln/value/rgb8.hh>
#include <mln/fun/v2v/rgb_to_int_u.hh>

#include <mln/linear/gaussian.hh>
#include <mln/util/array.hh>


#include "frontpropagate.hh"
#include "grouping.hh"
#include "display.hh"
#include "parametre.hh"

#define color 1
#define black 0
#define gradi 0
/*
 i=5;./StageAtEpitaCode/trunk/frontpropagate ~/Documents/ima/test_set/img_${i}.jpg 11 11 0Testoutput/lap${i}.ppm 0Testoutput/lab${i}.png 30 0Testoutput/bin${i}.pbm 0 0 1 1 0Testoutput/${i}.png 3.1 0Testoutput/Gau${i}.png 0Testoutput/gt.txt
 */
void help(char * argv[])
{
  
  std::cerr << "usage: " << argv[0] << " input.ext Laplace_window gradian_window gradient_thresholding gradient_output.ext ShowContour(1|0) elementBox(1|0) textBox(1|0) textBoxOnOriginalImage(1|0) textBox_On_originalImage.ext GauSigma BlurredImage.ext BoundingBox.txt " << std::endl;
  std::cerr << "or: " << argv[0] << " input.ext Laplace_window gradian_window gradient_thresholding gradient_output.ext ShowContour(1|0) elementBox(1|0) textBox(1|0) Color_binary(1|0) textBox_On_originalImage.ext GauSigma BlurredImage.ext BoundingBox.txt minContour minWidth minHeight widthHeightRatio heightWidthRatio " << std::endl;
  std::abort();

}

int main(int argc, char* argv[])
{
  using namespace std;
  using namespace mln;
  if (argc < 16 or argc >21)
    help(argv);
  
 if (argc ==21)
    {
      params::area = atoi(argv[16]);
      params::minWidth = atoi(argv[17]);
      params::minHeight = atoi(argv[18]);
      params::widthHeightRatio = atoi(argv[19]);
      params::heightWidthRatio = atoi(argv[20]);
    }

  util::timer t1,t2,t4,t5;
  t1.start();t2.start();t4.start();
  //input
  
  image2d<value::rgb8> inputrgb;
  io::magick::load(inputrgb,argv[1]);
  image2d<value::int_u8> input = tos::transformBnW(inputrgb);
    /*************************************/
  if(atof(argv[13])>0.001)
  {
    input = linear::gaussian(input, atof(argv[13]));
    io::magick::save(input,argv[14]);
  }

  //***********************************/
  //
  border::thickness = 1;
  std::cout <<" input and convert "<<t4.stop() * 1000. << " ms" << std::endl;

  t4.reset();
  t4.start();
  win::rectangle2d lapw( atoi(argv[2]),atoi(argv[2]));
  win::rectangle2d gradw( atoi(argv[3]), atoi(argv[3]));
  image2d<float> lap(input.domain());

  //Calculate Laplacian
  morpho::laplacian(input, lapw, lap);
  display::laplacian_colorization(lap,argv[4]);
  //display::laplacian_BlackNWhite(lap,argv[4])
  
  //Calculate gradient

  image2d<value::int_u8> grad = morpho::gradient(input,gradw);
  io::magick::save(grad,argv[7]);
  std::cout <<"lap and grad "<<t4.stop() * 1000. << " ms" << std::endl;
  t4.reset();
  t4.start();

  /*-----------------------------------------------------*/
  input = tos::add_border(input);
  lap = tos::add_border(lap);
  grad = tos::add_border(grad);
  /*-----------------------------------------------------*/
  //params::laplacianThreshold=atoi(argv[6]);

  std::cout <<"Immersed "<<t4.stop() * 1000. << " ms" << std::endl;

  t5.start();
  //init output value
  tos::tos treeOfShape(lap.nrows()*lap.ncols());
  util::timer t3;
  t3.start();
  image2d<unsigned int> output_4 = tos::labelling(input,lap,grad,atof(argv[6]),treeOfShape);//replace 30 = atof(argv[6]) (gradThresHold)
  std::cout <<"outside labelling "<< t3.stop() * 1000. << " ms" << std::endl;

  //get an output
  std::cout<<"number of Labels: "<<treeOfShape.nLabels<<endl;
  std::cout << t1.stop() * 1000. << " ms" << std::endl;
  t1.start();  
  //get the bounding box
  if(atoi(argv[10]))
    {
      treeOfShape.boundingBoxes = tos::three::getRegions(output_4,treeOfShape);
    }
  //get the bounding box and display on original image
  
  display::box_on_image(inputrgb,treeOfShape,argv[12]);
  //displace the labels with different color with color labels
  
  
  display::saveGT(treeOfShape,argv[15]);
  
  if(atoi(argv[11]))
  {display::label_colorization(output_4,argv[5],treeOfShape,atoi(argv[8]),atoi(argv[9]),atoi(argv[10]));}
  else  
  {display::label_OnlyText(output_4,argv[5],treeOfShape,atoi(argv[8]),atoi(argv[9]),atoi(argv[10]));}

  
  //cout<<treeOfShape.lambda.size()<<" "<<treeOfShape.nLabels<<" "<<treeOfShape.border.size()<<endl;
  std::cout << t2.stop() * 1000. << " ms" << std::endl;
  std::cout << t1.stop() * 1000. << " ms" << std::endl;
  
  std::cout <<"labelling and grouping "<< t5.stop() * 1000. << " ms" << std::endl;

  
}
//attention, the tree code lable from 1, but the vector starts from 0
