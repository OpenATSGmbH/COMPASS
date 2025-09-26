/*
 * AuctionAlgorithm.h
 *
 *  Created on: 30.07.2013
 *      Author: fb
 */

#ifndef AUCTIONALGORITHM_H_
#define AUCTIONALGORITHM_H_

#include <Eigen/Core>
#include <vector>

#define __AUCTION_EPSILON_MULTIPLIER 1e-5
#define __AUCTION_INF 1e8
//#define __AUCTION_OMIT_ZEROS
#define __AUCTION_ZERO 0.

template<typename Scalar = double>
class Auction
{
private:

    Auction() = delete;

    virtual ~Auction(){}
public:

    typedef Eigen::Matrix<Scalar, -1, -1> WeightMatrix;

    /**
     * solver modes for association problem
     */
    enum APSolvingMode
    {
        MAXIMIZATION = 1, MINIMIZATION = 2
    };

    /**
     * represents an undirected edge between node x and y with weight v
     */
    struct Edge
    {
    public:
        Edge() : x(0), y(0), v(0) {}
        Edge(const size_t x, const size_t y, const Scalar v) :
                x(x), y(y), v(v) {}

        size_t x;
        size_t y;
        Scalar v;
    };


    /**
     * vector of edges
     */
    typedef std::vector<Edge> Edges;


	/**
	 * vector of scalars (prices, profits, ...)
	 */
	typedef std::vector<Scalar> Scalars;

	/**
	 * vector of bools for row/column-locking
	 */
	typedef std::vector<bool> Locks;

	/**
	 * vector of indices
	 */
	typedef std::vector<size_t> indices;

	static const Edges solve(const Eigen::Matrix<Scalar, -1, -1> & a)
	{
		const size_t rows = a.rows();
		const size_t cols = a.cols();

		Locks lockedRows(a.rows(), false);
		Locks lockedCols(a.cols(), false);
		Edges E;

		Scalar lambda = .0;
		Scalar epsilon = __AUCTION_EPSILON_MULTIPLIER / a.cols();

		// condition 3: initially set p_j >= lambda
		Scalars prices(cols, 0.), profits(rows, 1.); // p-Vector  (1 to j) = p_j

		do
		{
			//		Step 1 (forward	auction cycle):
			//		Execute iterations of the forward auction algorithm until at least one
			//		more person becomes assigned. If there is an unassigned person left, go
			//		to step 2; else go to step 3.
			while (forward(a, E, prices, profits, lockedRows, lockedCols,
					lambda, epsilon))
				;

			if (!allPersonsAssigned(lockedRows))
			{

				//		Step 2 (reverse auction cycle):
				//		Execute several iterations of the reverse auction algorithm until at least
				//		one more object becomes assigned or until we have p_j <= lambda for all
				//		unassigned objects. If there is an unassigned person left, go to step 1
				//		else go to step 3
				while (!reverse(a, E, prices, profits, lockedRows, lockedCols,
						lambda, epsilon)
						|| !unassignedObjectsLTlambda(lockedCols, prices,
								lambda))
					; // reverse auction
			}

			if (allPersonsAssigned(lockedRows))
			{
				//		Step 3 (reverse auction):
				//		Execute successive iterations of the reverse auction algorithm until the
				//		algorithm terminates with p_j <= lambda for all unassigned objects j
				while (true)
				{
					reverse(a, E, prices, profits, lockedRows, lockedCols,
							lambda, epsilon);
					if (unassignedObjectsLTlambda(lockedCols, prices, lambda))
						break;
				}
				break;
			}

		} while (true);

		return E;
	}
private:

	/**
	 * forward cycle of auction algorithm
	 * @param a weight matrix (nxm)
	 * @param S assignment matrix (nxm)
	 * @param prices prices per object (m)
	 * @param profits profits per person (n)
	 * @param lambda bidding threshold lambda
	 * @param epsilon bidding increment
	 * @return true if assignment was made, false otherwise
	 */
	static bool forward(const Eigen::Matrix<Scalar, -1, -1> & a, Edges & E,
			Scalars & prices, Scalars & profits, Locks & lockedRows,
			Locks & lockedCols, Scalar & lambda, Scalar & epsilon)
	{
#ifdef __AUCTION_DEBUG
		__A_FORWARD_LOG << "forwarding ..." << std::endl;
#endif
		const size_t rows = a.rows();
		const size_t cols = a.cols();
		bool assignmentFound = false;

		for (size_t i = 0; i < rows; i++) // for the i-th row/person
		{
#ifdef __AUCTION_DEBUG
			__A_FORWARD_LOG << "examining row " << i << std::endl;
#endif
			bool assignmentInThisIterationFound = false;

			// person already assigned?
			if (lockedRows[i])
				continue;

#ifdef __AUCTION_DEBUG
			__A_FORWARD_LOG << "row " << i << " not locked!" << std::endl;
#endif
			// find an unassigned person i, its best object j_i
			// j_i = argmax {a_ij - p_j} for j in A(i) ( A(i) are the set of edges of the i-th row )
			// if a(i,j) = 0. it is not a valid edge
			size_t j_i = 0;

			//	v_i = max { a_ij - p_j} for j in A(i)				// maximum profit for person i
			//  v_i was already found = v_i
			//	w_i = max { a_ij - p_j} for j in A(i) and j != j_i  // second best profit
			//	if j_i is the only entry in A(i), w_i = - inf       // there's no second best profit
			Scalar w_i = -__AUCTION_INF, v_i = -__AUCTION_INF, a_i_ji = 0.;	// = max { a_ij - p_j}

			// find maximum profit i.e. j_i = arg max { a_ij - p_j} and second best
			for (size_t j = 0; j < cols; j++) // for the j-th column
			{
				const Scalar aij = a(i,j);
#ifndef __AUCTION_OMIT_ZEROS
				if ( aij == __AUCTION_ZERO ) continue;
#endif
				const Scalar diff = aij - prices[j];
#ifdef __AUCTION_DEBUG
			__A_FORWARD_LOG << "  col " << j << " diff = " << diff << std::endl;
#endif
				if (diff > v_i)
				{
#ifdef __AUCTION_DEBUG
			__A_FORWARD_LOG << "  diff > v_i !" << std::endl;
#endif
					// if there already was an entry found, this is the second best
					if (assignmentInThisIterationFound)
						w_i = v_i;

					v_i = diff;
					j_i = j;
					a_i_ji = aij;
					assignmentInThisIterationFound = true;
				}
				if (diff > w_i && j_i != j)
					w_i = diff;
				// if no entry is bigger than v_i, check if there's still a bigger second best entry
			}

			// no possible assignment found?
			if (!assignmentInThisIterationFound)
			{
				lockedRows[i] = true;	// if no assignment found in this row, there is no arc ...
				continue;
			}
#ifdef __AUCTION_DEBUG
			__A_FORWARD_LOG << "i = " << i << " - j_i = " << j_i << " - v_i = " << v_i
			<< " - w_i = " << w_i << " - a_ij = " << a_i_ji << std::endl;
#endif
			assignmentInThisIterationFound = false;

//			std::cout << "assignment found .." << std::endl;
			const Scalar bid = a_i_ji - w_i + epsilon;

			//	P_i = w_i - E
			profits[i] = w_i - epsilon; // set new profit for person

			//	prices(j_i) = max(lambda, a(i,j_i) - w(i) + epsilon)
			// if lambda <= a_ij - w_i + E, add (i, j_i) to S
			if (lambda <= bid)
			{
				prices[j_i] = bid;
				// assignment was made, so lock row and col
				lockedRows[i] = true;
				lockedCols[j_i] = true;

				bool newEdge = true;

				// if j_i was assigned to different i' to begin, remove (i', j_i) from S
				for (auto & e : E)
					if (e.y == j_i) // change edge
					{
						lockedRows[e.x] = false; // unlock row i'
						newEdge = false;
						e.x = i;
						e.v = a_i_ji;
						break;
					}
				if (newEdge)
				{
                    Edge e;
					e.x = i;
					e.y = j_i;
					e.v = a_i_ji;
					E.push_back(e);
#ifdef __AUCTION_DEBUG
		__A_FORWARD_LOG << "adding edge (" << i << ", " << j_i << ")" << std::endl;
#endif
				}
				assignmentInThisIterationFound = true;

			}
			else
			{
				prices[j_i] = lambda;
				assignmentInThisIterationFound = false;

			}
			if (assignmentInThisIterationFound)
				assignmentFound = true;
		}
		return assignmentFound;

	}

	/**
	 * reverse cycle of auction algorithm
	 * @param a weight matrix (nxm)
	 * @param S assignment matrix (nxm)
	 * @param prices prices per object (m)
	 * @param profits profits per person (n)
	 * @param lambda bidding threshold lambda
	 * @param epsilon bidding increment
	 * @return true if assignment was made, false otherwise
	 */
	static bool reverse(const Eigen::Matrix<Scalar, -1, -1> & a, Edges & E,
			Scalars & prices, Scalars & profits, Locks & lockedRows,
			Locks & lockedCols, Scalar & lambda, const Scalar & epsilon)
	{
#ifdef __AUCTION_DEBUG
		__A_REVERSE_LOG << "reversing ..." << std::endl;
#endif
		const size_t rows = a.rows();
		const size_t cols = a.cols();

		bool assignmentFound = false;

		for (size_t j = 0; j < cols; j++) // for the j-th column (objects)
		{
			bool assignmentInThisIterationFound = false;

			// object already assigned,  p_j > lambda ?
			if (lockedCols[j])
				continue;

			if (!(prices[j] > lambda))
				continue;

			// Find an unassigned object j with p_j > lambda, its best person i_j
			// i_j = argmax {a_ij - profits[i]) für i aus B(j) (PI !!!)
			size_t i_j = 0;

			//g_j = max {a_ij - P_i} for i in B(j) and i != i_j
			// if j_i is the only entry in B(j), g_j = - inf ( g_j < b_j)
			//b_j = max {a_ij - P_i} for i in B(j)
			Scalar b_j = -__AUCTION_INF, g_j = -__AUCTION_INF, a_ij_j = 0.;

			// find maximum profit i.e. j_i = arg max { a_ij - p_j} and second best
			for (size_t i = 0; i < rows; i++) // for the j-th column
			{
				const Scalar aij = a(i, j);
#ifndef __AUCTION_OMIT_ZEROS
				if ( aij == __AUCTION_ZERO ) continue;
#endif
				const Scalar diff = aij - profits[i];
				if (diff > b_j)
				{
					// if there already was an entry found, this is the second best
					if (assignmentInThisIterationFound)
						g_j = b_j;

					b_j = diff;
					i_j = i;
					a_ij_j = aij;
					assignmentInThisIterationFound = true;
				}
				if (diff > g_j && i_j != i)
					g_j = diff;
			}

			// no assignment found
			if (!assignmentInThisIterationFound)
			{
				lockedCols[j] = true;
				continue;
			}
#ifdef __AUCTION_DEBUG
			__A_REVERSE_LOG << "j = " << j << " i_j = " << i_j << " b_j = " << b_j << " g_j = " << g_j
			<< " a_ij_j = " << a_ij_j
			<< " p_j = " << prices[j] << " P_i = " << profits[i_j]<< std::endl;
#endif
			assignmentInThisIterationFound = false;

			//if b_j >= L + E, case 1:
			if (b_j >= (lambda + epsilon))
			{
#ifdef __AUCTION_DEBUG
				__A_REVERSE_LOG << "b_j >= lambda + epsilon" << std::endl;
#endif
				const Scalar diff = g_j - epsilon; // G_j - E

				const Scalar max = lambda > diff ? lambda : diff; //  max { L, G_j - E}

				//	p_j = max { L, G_j - E}
				prices[j] = max;

				//	P_i_j = a_i_jj - max {L, G_j - E}
				profits[i_j] = a_ij_j - max;

				lockedRows[i_j] = true;
				lockedCols[j] = true;

				bool newEdge = true;

				// if j_i was assigned to different i' to begin, remove (i', j_i) from S
				for (auto & e : E)
					if (e.x == i_j) // change edge
					{
						lockedCols[e.y] = false; // unlock row i'
						newEdge = false;
						e.y = j;
						e.v = a_ij_j;
#ifdef __AUCTION_DEBUG
						__A_REVERSE_LOG << "edges: " << E.size()
						<< "  changing edge ";
#endif
						break;

					}
				if (newEdge)
				{
                    Edge e;
					e.x = i_j;
					e.y = j;
					e.v = a_ij_j;
					E.push_back(e);
#ifdef __AUCTION_DEBUG
					__A_REVERSE_LOG << "added edge " << E.size() << " ";
#endif
				}
				assignmentInThisIterationFound = true;
			}
			else	// if B_j < L + E, case 2
			{
				//	p_j = B_j - E
				prices[j] = b_j - epsilon;
#ifdef __AUCTION_DEBUG
				__A_REVERSE_LOG << "b_j < lambda + epsilon " << std::endl;
#endif
				/** standard lambda scaling **/
				size_t lowerThanLambda = 0;
				Scalar newLambda = lambda;

				// if the number of objectes k with p_k < lambda is bigger than (rows - cols)
				for (size_t k = 0; k < cols; k++)
				{
					if (prices[k] < lambda) // p_k < lambda
					{
						lowerThanLambda++;
						if (prices[k] < newLambda)
							newLambda = prices[k];
					}
				}
				// set new lambda
#ifdef __AUCTION_DEBUG
				__A_REVERSE_LOG << "changing lambda from " << lambda << " to " << newLambda << std::endl;
#endif
				if (lowerThanLambda >= (cols - rows))
					lambda = newLambda;
				assignmentInThisIterationFound = false;
			}
			if (assignmentInThisIterationFound)
				assignmentFound = true;
		}
		return assignmentFound;
	}


	/**
	 * returns true if p_j <= lambda for all unassigned objects.
	 *
	 * @param c locked columns
	 * @param prices prices of objects
	 * @param lambda bidding threshold
	 * @return true if all prices of unassigned objects are below lambda, otherwise false
	 */
	static const bool unassignedObjectsLTlambda(const Locks & c,
			const Scalars & prices, const Scalar lambda)
	{
		for (size_t j = 0; j < c.size(); ++j)
			if (!c[j] && prices[j] > lambda)
				return false;

		return true;
	}


	/**
	 * check if all persons are assigned
	 * @return true if all persons are assigned, otherwise false
	 */
	static const bool allPersonsAssigned(const Locks & r)
	{
		for (size_t i = 0; i < r.size(); ++i)
			if (!r[i])
				return false;
		return true;
	}
};


#endif /* AUCTIONALGORITHM_H_ */
