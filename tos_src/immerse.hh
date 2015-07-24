#ifndef IMMERSE_HH
# define IMMERSE_HH

# include <vector>
# include <algorithm>

# include <mln/core/image/image2d.hh>
# include <mln/core/alias/neighb2d.hh>
# include <mln/value/int_u8.hh>
# include <mln/data/paste.hh>
# include <mln/draw/box.hh>

# include "range.hh"
# include "faces.hh"



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



    // subdivision

    template <typename I >
    image2d< I >
    subdivide(const image2d< I >& input)
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
	    output(p) = std::max(input.at_(row / 2, (col - 1) / 2),
				 input.at_(row / 2, (col + 1) / 2));
	  else if (p.col() % 2 == 0)
	    // case c
	    output(p) = std::max(input.at_((row - 1) / 2, col / 2),
				 input.at_((row + 1) / 2, col / 2));
	  else
	    // case d
	    output(p) = std::max(std::max(input.at_((row - 1) / 2, (col - 1) / 2),
					  input.at_((row - 1) / 2, (col + 1) / 2)),
				 std::max(input.at_((row + 1) / 2, (col - 1) / 2),
					  input.at_((row + 1) / 2, (col + 1) / 2)));

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


    // addition of k-faces

    template <typename I >
    image2d< range<I> >
    add_kfaces(const image2d<I>& input)
    {
      box2d
	b = input.domain(),
	bb(2 * b.pmin() - dpoint2d(1,1),
	   2 * b.pmax() + dpoint2d(1,1));

      //   0 1
      // 0 a b
      // 1 c d

      // ->

      //  -1 0 1 2 3
      //-1 + - + - +
      // 0 | a | b |
      // 1 + - + - +
      // 2 | c | d |
      // 3 + - + - +

      typedef range<I> R;
      image2d<R> output(bb);
      I min, max;

      mln_piter(box2d) p(bb);
      mln_niter(neighb2d) n(c8(), p);
      for_all(p)
	if (face_dim(p) == 2)
	  output(p) = input.at_(p.row() / 2, p.col() / 2);
	else
	  {
	    min = 255;
	    max = 0;
	    for_all(n)
	      if (bb.has(n) and face_dim(n) == 2)
	      {
		I v = input.at_(n.row() / 2, n.col() / 2);
		if (v < min)
		  min = v;
		if (v > max)
		  max = v;
	      }
	    output(p) = R(min, max);
	  }

      return output;
    }

    template <typename I >
    image2d< range<I> >
    convert_to_range(const image2d<I>& input)
    {
      typedef range<I> R;
      image2d<R> output(input.domain());
      mln_piter(box2d) p(input.domain());
      for_all(p)
	output(p) = input(p);

      return output;
    }

    // immersion
    template <typename I >
    image2d< range<I> >
    immerse(const image2d<I>& input)
    {
      image2d<I> temp = add_border(input);
      temp = subdivide(temp);
      return add_kfaces(temp);
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
