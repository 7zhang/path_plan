/*
 * Copyright (c) 2011 Adrian Michel
 * http://www.amichel.com
 *
 * Permission to use, copy, modify, distribute and sell this 
 * software and its documentation for any purpose is hereby 
 * granted without fee, provided that both the above copyright 
 * notice and this permission notice appear in all copies and in 
 * the supporting documentation. 
 *  
 * This library is distributed in the hope that it will be 
 * useful. However, Adrian Michel makes no representations about
 * the suitability of this software for any purpose.  It is 
 * provided "as is" without any express or implied warranty. 
 * 
 * Should you find this library useful, please email 
 * info@amichel.com with a link or other reference 
 * to your work. 
*/

#ifndef DE_INDIVIDUAL_HPP_INCLUDED
#define DE_INDIVIDUAL_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include <queue>
#include <limits>
#include <cmath>

#include "de_types.hpp"
#include "de_constraints.hpp"
#include "multithread.hpp"
#include "system_state.h"

namespace de
{
class individual;
typedef boost::shared_ptr< individual > individual_ptr;

/**
 * An individual of a Differrential Evolution population 
 *  
 * An individual has a set of variables and a cost associated 
 * with these variables 
 *  
 * An individual is thread safe 
 * 
 * @author adrian (12/1/2011)
 */

class individual
{
private:
	de::DVectorPtr m_vars;
//	 m_s;
	double m_cost;
//	de::mutex m_mx;

public:
	/**
	 * constructs an individual
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param varCount the number of variables for each individual
	 */
	individual( size_t varCount )
	: m_vars( boost::make_shared< de::DVector >( varCount ) )
	{
	}

	/**
	 * constructs an individual
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param vars a vector of variables that will be copied into 
	 *  		   the internal vector of variables
	 */
	individual( const de::DVector& vars )
		: m_vars( boost::make_shared< de::DVector >( vars ) )
	{
	}

	/**
	 * Initialized the internal vector of variables with random 
	 * values within the constraints 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param constraints 
	 */
	void init( constraints_ptr constraints )
	{
		assert( constraints );
		assert( m_vars );
		assert( m_vars->size() == constraints->size() );

		for( de::DVector::size_type j = 0; j < m_vars->size(); ++j ) 
			(*m_vars)[ j ] = constraints->get_rand_value( j );
	}

	/**
	 * returns the cost
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @return double 
	 */
	double cost() const { return m_cost; }
//	T& get_state() { return m_s; }

	/**
	 * Sets the variables to new random values within the 
	 * constraints, using origin and old value 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param constraints 
	 * @param origin 
	 */
	void ensureConstraints( constraints_ptr constraints, de::DVectorPtr origin )
	{
		assert( constraints );
		assert( m_vars );
		assert( origin );
		assert( m_vars->size() == constraints->size() );

		for( de::DVector::size_type j = 0; j < m_vars->size(); ++j )
		{
			(*m_vars)[ j ] = constraints->get_rand_value( j, (*m_vars)[ j ], (*origin)[ j ] );
		}
	}

	/**
	 * returns the internal variable vector 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @return de::DVectorPtr 
	 */
	de::DVectorPtr vars() const { return m_vars; }

	/**
	 * returns a non constant reference to a variable value 
	 * based on an index, which can be used as an lvalue. 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param index the index of the variable whose reference to 
	 *  			return
	 * 
	 * @return de::Double& 
	 */
	de::Double& operator[]( size_t index ) { return (*m_vars)[ index ]; }

	/**
	 * returns a constant reference to a variable value based 
	 * on an index, which can be used as an lvalue. 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param index the index of the variable whose reference to return
	 * 
	 * @return de::Double& 
	 */
	const de::Double& operator[]( size_t index ) const { return (*m_vars)[ index ]; }

	individual& operator=(const individual &rhs) {
		*m_vars = *rhs.m_vars;
		m_cost = rhs.m_cost;
	}
	/**
	 * Sets the cost
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param cost 
	 */
	void setCost( double cost ) { m_cost = cost; }

	/**
	 * compares the current individual with another individual based 
	 * on cost. Returns true if the current cost is lower or equal 
	 * to the cost of the other individual 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param ind the individual to compare with
	 * 
	 * @return bool 
	 */
	bool operator<=( const individual& ind ) const
	{
		assert( ind.size() == size() );
		return cost() <= ind.cost();
	}

	/**
	 * compares the current individual with another individual based 
	 * on cost. Returns true if the current cost is strictly lower 
	 * than the cost of the other individual 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param ind the individual to compare with
	 * 
	 * @return bool 
	 */
	bool operator<( const individual& ind ) const
	{
		assert( ind.size() == size() );
		return cost() < ind.cost();
	}

	/**
	 * compares the current individual with another individual based 
	 * on cost. Returns true if the current cost is better or equal 
	 * than the cost of the other individual, where better is either
	 * lower or higher depending on the minimize flag 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param ind the individual to compare with 
	 * @param minimize <= if true, >= if false
	 * 
	 * @return bool 
	 */
	bool better_or_equal( const individual_ptr ind, bool minimize ) const
	{
		assert( ind );
//		double tmp = std::numeric_limits<double>::quiet_NaN();
		if (!std::isnan(m_cost) && !std::isnan(ind->cost())) {
//			std::cout << "branch 1: " << m_cost << " " << ind->cost() << std::endl;
			return minimize ? *this <= *ind : *ind <= *this;
		} else if (!std::isnan(ind->cost())) {
//			std::cout << "branch 2: " << m_cost << " " << ind->cost() << std::endl;
			return false;
		} else {
//			std::cout << "branch 3: " << m_cost << " " << ind->cost() << std::endl;
			return true;
		}
	}

	/**
	 * compares the current individual with another individual based 
	 * on cost. Returns true if the current cost is strictly better 
	 * than the cost of the other individual, where better is either
	 * lower or higher depending on the minimize flag 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param ind the individual to compare with 
	 * @param minimize < if true, > if false 
	 * 
	 * @return bool 
	 */
	bool better( const individual_ptr ind, bool minimize ) const
	{
		better_or_equal(ind, minimize);
		// assert( ind );
		// return minimize ? *this < *ind : *ind < *this;
	}

	/**
	 * returns the size of variables vector
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @return size_t 
	 */
	size_t size() const { return m_vars->size(); }

	/**
	 * returns a string representation of the internals of an 
	 * individual (cost and list of variables)
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @return std::string 
	 */
	std::string to_string() const
	{
		std::ostringstream os;
//		os.precision(16);
//		os << std::setw(10);
		os.width(10);

		//os << "cost: " << cost() << ", vars: ";
		os << cost() << " ";

		for( de::DVector::size_type j = 0; j < m_vars->size(); ++j )
		{
			os.width(10);
			os << (*m_vars)[ j ] << " ";
		}

		return os.str();

	}
};

typedef std::queue< individual_ptr > individual_queue_base;

/**
 * A thread safe queue of individuals 
 *  
 * Used to queue Individuals containing the arguments to be 
 * passed by different processors to the objective function 
 * 
 * @author adrian (12/1/2011)
 */
class individual_queue : public individual_queue_base
{
private:
 	de::mutex m_mx;

public:
	/**
	 * adds a new individual at the bottom of the queue 
	 *  
	 * is thread safe 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param ind the individual to insert into the queue
	 */
	void push_back( individual_ptr ind )
	{
		de::lock lock( m_mx );

		individual_queue_base::push( ind );
	}

	/**
	 * removes the individual from the top of the queue (if any) and
	 * returns it 
	 * 
	 * is thread safe 
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @return individual_ptr the individual at the top of the queue 
	 *  	   or null individual if the queue is empty
	 */
	individual_ptr pop()
	{
		de::lock lock( m_mx );

		if( !individual_queue_base::empty() )
		{
			individual_ptr p( individual_queue_base::front() );

			individual_queue_base::pop();

			return p;
		}
		else
			return individual_ptr();
	}
};

}

#endif //DE_INDIVIDUAL_HPP_INCLUDED