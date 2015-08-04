#ifndef IMMERSE_HH
# define IMMERSE_HH

# include <vector>
# include <algorithm>

# include <mln/core/image/image2d.hh>
# include <mln/core/alias/neighb2d.hh>
# include <mln/value/int_u8.hh>
# include <mln/data/paste.hh>
# include <mln/draw/box.hh>



namespace mln
{

  namespace tos
  {

    namespace internal
    {
      template <typename I>
      I compute_median(I v1,I v2,I v3,I v4)
      {
	std::vector<I> val;
	val.push_back(v1);
	val.push_back(v2);
	val.push_back(v3);
	val.push_back(v4);
	std::sort(val.begin(),val.end());
	return (val[1]+val[2])/2;
      }
    }
    
    
    // add_border

    template <typename I >
    I compute_border_median(const image2d<I>& input)
    {
      box2d b = input.domain();
      const short
	max_row = b.pmax().row(),
	max_col = b.pmax().col();

      std::vector<I> val;
      unsigned n = 2 * b.nrows() + 2 * b.ncols() - 4;
      val.reserve(n);

      for (short col = 0; col <= max_col; ++col)
	{
	  val.push_back(input.at_(0, col));
	  val.push_back(input.at_(max_row, col));
	}
      for (short row = 1; row < max_row; ++row)
	{
	  val.push_back(input.at_(row, 0));
	  val.push_back(input.at_(row, max_col));
	}

      std::sort(val.begin(), val.end());
      return val[n/2];
    }

    template <typename I >
    image2d<I>
    add_border(const image2d< I >& input)
    {
      box2d b = input.domain(), bb = b;
      bb.enlarge(1);
      image2d< I > output(bb);
      
      data::paste(input, output); // in b, output is input

      I m = compute_border_median(input);
      draw::box(output, bb, m); // otherwise, output is m on the border

      return output;
    }



    template <typename I >
    image2d< I >
    subdivide2(const image2d< I >& input)
    {
      box2d
	b = input.domain(),
	bb(2 * b.pmin(), 2 * b.pmax());
      image2d< I > output(bb);

      //   0 1
      // 0 a a
      // 1 a a

      // ->

      //   0 1 2
      // 0 a b a
      // 1 c d c
      // 2 a b a

      point2d p;
      short& row = p.row();
      short& col = p.col();

      for (row = bb.pmin().row(); row <= bb.pmax().row(); ++row)
	for (col = bb.pmin().col(); col <= bb.pmax().col(); ++col)

	  if (p.row() % 2 == 0 and p.col() % 2 == 0)
	    // case a
	    output(p) = input.at_(row / 2, col / 2);
	  else if (p.row() % 2 == 0)
	    // case b
	    output(p) = ( (input.at_(row / 2, (col - 1) / 2) +  input.at_(row / 2, (col + 1) / 2)) )/2;
	  else if (p.col() % 2 == 0)
	    // case c
	    output(p) = ((input.at_((row - 1) / 2, col / 2)+ input.at_((row + 1) / 2, col / 2)) )/2;
	  else
	    // case d
	    output(p) = internal::compute_median(input.at_((row - 1) / 2, (col - 1) / 2) , input.at_((row - 1) / 2, (col + 1) / 2),
						   input.at_((row + 1) / 2, (col - 1) / 2),  input.at_((row + 1) / 2, (col + 1) / 2));

      return output;
    }    


    template <typename I >
    image2d< I >
    immerse2(const image2d<I>& input)
    {
      image2d<I> temp = add_border(input);
      return subdivide2(temp);
    }


  } // end of namespace tos

} // end of namespace mln


#endif // ndef IMMERSE_HH
