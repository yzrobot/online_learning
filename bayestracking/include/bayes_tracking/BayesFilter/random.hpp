/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Good random numbers from Boost
 *  Provides a common class  for all random number requirements to test Bayes++
 */

#ifndef Bayesian_filter_RANDOM_HPP
#define Bayesian_filter_RANDOM_HPP

#include <boost/version.hpp>
#include <boost/random.hpp>


namespace Bayesian_filter_test
{

namespace
{
	template<class Engine, class Distribution>
	class simple_generator
	{
	public:
		typedef typename Distribution::result_type result_type;
		simple_generator(Engine& e, Distribution& d)
			: _eng(e), _dist(d)
		{}
		result_type operator()()
		{	return _dist(_eng);
		 }
	private:
		Engine& _eng;
		Distribution& _dist;
	};
}//namespace

class Boost_random
{
public:
	typedef Bayesian_filter_matrix::Float Float;
	typedef boost::mt19937 URng;
	Boost_random() : dist_uniform_01(), dist_normal_01()
	{}
	Bayesian_filter_matrix::Float normal(const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		return dist(rng);
	}
	void normal(Bayesian_filter_matrix::DenseVec& v, const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		simple_generator<URng, boost::normal_distribution<Float> > gen(rng, dist);
		std::generate (v.begin(), v.end(), gen);
	}
	void normal(Bayesian_filter_matrix::DenseVec& v)
	{
		simple_generator<URng, boost::normal_distribution<Float> > gen(rng, dist_normal_01);
		std::generate (v.begin(), v.end(), gen);
	}
	void uniform_01(Bayesian_filter_matrix::DenseVec& v)
	{
		simple_generator<URng, boost::uniform_01<Float> > gen(rng, dist_uniform_01);
		std::generate (v.begin(), v.end(), gen);
	}
#ifdef BAYES_FILTER_GAPPY
	void normal(Bayesian_filter_matrix::Vec& v, const Float mean, const Float sigma)
	{
		boost::normal_distribution<Float> dist(mean, sigma);
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
	void normal(Bayesian_filter_matrix::Vec& v)
	{
		simple_generator_01<URng, boost::normal_distribution<Float> > gen(rng, dist_normal_01);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
	void uniform_01(Bayesian_filter_matrix::Vec& v)
	{
		simple_generator<URng, boost::uniform_01<Float> > gen(rng, dist_uniform_01);
		for (std::size_t i = 0, iend=v.size(); i < iend; ++i)
			v[i] = gen();
	}
#endif
	void seed()
	{
		rng.seed();
	}
private:
	URng rng;
	boost::uniform_01<Float> dist_uniform_01;
	boost::normal_distribution<Float> dist_normal_01;
};


}//namespace

#endif
