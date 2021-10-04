/*
*   Name        : random_generator.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for generate random sampling candidates
*   Availability:
*   Copyright   : Copyright (C) 2021 by Weixiao GAO (gaoweixiaocuhk@gmail.com)
*                 All rights reserved.
*
*				  This file is part of semantic_urban_mesh_segmentation: software
*				  for semantic segmentation of textured urban meshes.
*
*				  semantic_urban_mesh_segmentation is free software; you can
*				  redistribute it and/or modify it under the terms of the GNU
*				  General Public License Version 3 as published by the Free
*				  Software Foundation.
*
*				  semantic_urban_mesh_segmentation is distributed in the hope that
*				  it will be useful, but WITHOUT ANY WARRANTY; without even the
*				  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
*				  PURPOSE. See the GNU General Public License for more details.
*
*				  You should have received a copy of the GNU General Public License
*				  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#ifndef semantic_mesh_segmentation__RANDOM_GENERATOR_HPP
#define semantic_mesh_segmentation__RANDOM_GENERATOR_HPP

#include <easy3d/point_cloud.h>

namespace semantic_mesh_segmentation 
{
	namespace random_generator
	{
		/**
		 * Common interface for random generation (with uniform distribution).
		 *
		 * Two RNGs are available: Subtractive Ring and an improved Marsenne-Twister.
		 */

		class RandomGenerator
		{
			// construction
		public:
			RandomGenerator() {}

			virtual ~RandomGenerator()
			{}

			// public methods
		public:
			/// (Re-)initialize with a given seed.
			virtual void initialize(unsigned int seed) = 0;

			/// Return a random number in the given range (note that not all the RNG can handle a given limit).
			virtual unsigned int generate(unsigned int limit) = 0;

			/// Return a random number in the [0,1) real interval.
			virtual double generate01() = 0;

			/// Returns a random number in the [0,1] real interval.
			virtual double generate01closed() = 0;

			/// Generates a random number in the (0,1) real interval.
			virtual double generate01open() = 0;
		};

		/**
		 * The second one is an improved Marsenne-Twister algorithm (MT19937)
		 * Coded by Takuji Nishimura and Makoto Matsumoto (see copyright note below)
		 * and successively modified to be a C++ class by Daniel Dunbar.
		 *
		 *
		 * References for improved Marsenne-Twister:
		 *
		 *   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
		 *
		 */
		class MarsenneTwisterRNG : public RandomGenerator
		{
			// definitions
		private:
			static const int N = 624;
			static const int M = 397;
			static const unsigned int MATRIX_A = 0x9908b0dfu;   // constant vector a
			static const unsigned int UPPER_MASK = 0x80000000u; // most significant w-r bits
			static const unsigned int LOWER_MASK = 0x7fffffffu; // least significant r bits

		// private data member
		private:
			// Improved Marsenne-Twister RNG status variables
			unsigned int mt[N]; // the array for the state vector
			int mti;
			// construction
		public:
			// ctor
			MarsenneTwisterRNG()
			{
				initialize(5489u);
			}

			MarsenneTwisterRNG(unsigned int seed)
			{
				initialize(seed);
			}

			virtual ~MarsenneTwisterRNG()
			{}

			// public methods
		public:
			/// (Re-)initialize with the given seed.
			void initialize(unsigned int seed)
			{
				mt[0] = seed & 0xffffffffu;
				for (mti = 1; mti < N; mti++)
				{
					mt[mti] = (1812433253u * (mt[mti - 1] ^ (mt[mti - 1] >> 30)) + mti);

					/* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
					/* In the previous versions, MSBs of the seed affect   */
					/* only MSBs of the array mt[].                        */

					mt[mti] &= 0xffffffffu;

					/* for >32 bit machines */
				}
			}

			/**
			 * Initialize by an array with array-length.
			 *
			 * init_key is the array for initializing keys
			 * key_length is its length
			 */

			void initializeByArray(unsigned int init_key[], int key_length)
			{
				int i, j, k;
				initialize(19650218u);
				i = 1; j = 0;
				k = (N > key_length ? N : key_length);
				for (; k; k--)
				{
					mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1664525u)) + init_key[j] + j; /* non linear */
					mt[i] &= 0xffffffffu; /* for WORDSIZE > 32 machines */
					i++; 
					j++;

					if (i >= N)
					{
						mt[0] = mt[N - 1];
						i = 1;
					}

					if (j >= key_length) j = 0;
				}

				for (k = N - 1; k; k--)
				{
					mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1566083941u)) - i; /* non linear */
					mt[i] &= 0xffffffffu; /* for WORDSIZE > 32 machines */

					i++;

					if (i >= N)
					{
						mt[0] = mt[N - 1];
						i = 1;
					}
				}
				mt[0] = 0x80000000u; /* MSB is 1; assuring non-zero initial array */
			}

			unsigned int generate(unsigned int limit)
			{
				return generate() % limit;
			}

			/**
			 * Return a random number in the [0,0xffffffff] interval using the improved Marsenne Twister algorithm.
			 *
			 * NOTE: Limit is not considered, the interval is fixed.
			 */
			unsigned int generate()
			{
				unsigned int y;
				static unsigned int mag01[2] = { 0x0u, MATRIX_A };
				/* mag01[x] = x * MATRIX_A  for x=0,1 */

				if (mti >= N) // generate N words at one time
				{
					int kk;
					for (kk = 0; kk < N - M; kk++)
					{
						y = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
						mt[kk] = mt[kk + M] ^ (y >> 1) ^ mag01[y & 0x1u];
					}

					for (; kk < N - 1; kk++)
					{
						y = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
						mt[kk] = mt[kk + (M - N)] ^ (y >> 1) ^ mag01[y & 0x1u];
					}

					y = (mt[N - 1] & UPPER_MASK) | (mt[0] & LOWER_MASK);
					mt[N - 1] = mt[M - 1] ^ (y >> 1) ^ mag01[y & 0x1u];
					mti = 0;
				}

				y = mt[mti++];

				/* Tempering */
				y ^= (y >> 11);
				y ^= (y << 7) & 0x9d2c5680u;
				y ^= (y << 15) & 0xefc60000u;
				y ^= (y >> 18);

				return y;
			}

			/// Returns a random number in the [0,1] real interval using the improved Marsenne-Twister.
			double generate01closed()
			{
				return generate()*(1.0 / 4294967295.0);
			}

			/// Returns a random number in the [0,1) real interval using the improved Marsenne-Twister.
			double generate01()
			{
				return generate()*(1.0 / 4294967296.0);
			}

			/// Generates a random number in the (0,1) real interval using the improved Marsenne-Twister.
			double generate01open()
			{
				return (((double)generate()) + 0.5)*(1.0 / 4294967296.0);
			}

		}; // end class MarsenneTwisterRNG


		//  \brief Generate the barycentric coords of a random point over a single face,
		//  with a uniform distribution over the triangle.
		// It uses the parallelogram folding trick.
		inline easy3d::vec3 GenerateBarycentricUniform(MarsenneTwisterRNG &rnd)
		{
			easy3d::vec3 interp;

			interp[1] = rnd.generate01();
			interp[2] = rnd.generate01();

			if (interp[1] + interp[2] > 1.0)
			{
				interp[1] = 1.0 - interp[1];
				interp[2] = 1.0 - interp[2];
			}
			assert(interp[1] + interp[2] <= 1.0);
			interp[0] = 1.0 - (interp[1] + interp[2]);

			return interp;
		}

		// Generate the barycentric coords of a random point over a single face,
		// with a uniform distribution over the triangle.
		// It uses the parallelogram folding trick.
		static easy3d::vec3 RandomBarycentric(random_generator::MarsenneTwisterRNG &rng)
		{
			return GenerateBarycentricUniform(rng);
		}


#define FAK_LEN 1024
		static double LnFac(int n) 
		{
			// Tabled log factorial function. gives natural logarithm of n!
			// define constants
			static const double                 // coefficients in Stirling approximation
				C0 = 0.918938533204672722,      // ln(sqrt(2*pi))
				C1 = 1. / 12.,
				C3 = -1. / 360.;

			// C5 =  1./1260.,                  // use r^5 term if FAK_LEN < 50
			// C7 = -1./1680.;                  // use r^7 term if FAK_LEN < 20
			// static variables

			static double fac_table[FAK_LEN];   // table of ln(n!):
			static bool initialized = false;         // remember if fac_table has been initialized

			if (n < FAK_LEN) 
			{
				if (n <= 1) 
				{
					if (n < 0) assert(0);//("Parameter negative in LnFac function");
					return 0;
				}

				if (!initialized)
				{              // first time. Must initialize table
				   // make table of ln(n!)

					double sum = fac_table[0] = 0.;
					for (int i = 1; i < FAK_LEN; i++) 
					{
						sum += log(double(i));
						fac_table[i] = sum;
					}
					initialized = true;
				}

				return fac_table[n];
			}

			// not found in table. use Stirling approximation

			double  n1, r;
			n1 = n;  r = 1. / n1;
			return (n1 + 0.5)*log(n1) - n1 + C0 + r * (C1 + r * r*C3);
		}

		static int PoissonRatioUniforms(double L) 
		{
			/*
			This subfunction generates a integer with the poisson
			distribution using the ratio-of-uniforms rejection method (PRUAt).
			This approach is STABLE even for large L (e.g. it does not suffer from the overflow limit of the classical Knuth implementation)
			Execution time does not depend on L, except that it matters whether
			is within the range where ln(n!) is tabulated.
			Reference:
			E. Stadlober
			"The ratio of uniforms approach for generating discrete random variates".
			Journal of Computational and Applied Mathematics,
			vol. 31, no. 1, 1990, pp. 181-189.
			Partially adapted/inspired from some subfunctions of the Agner Fog stocc library ( www.agner.org/random )
			Same licensing scheme.
			*/

			// constants
			const double SHAT1 = 2.943035529371538573;    // 8/e
			const double SHAT2 = 0.8989161620588987408;   // 3-sqrt(12/e)
			double u;                                          // uniform random
			double lf;                                         // ln(f(x))
			double x;                                          // real sample
			int k;                                         // integer sample

			double   pois_a = L + 0.5;                               // hat center
			int mode = (int)L;                      // mode
			double   pois_g = log(L);
			double    pois_f0 = mode * pois_g - LnFac(mode);          // value at mode
			double   pois_h = sqrt(SHAT1 * (L + 0.5)) + SHAT2;         // hat width
			double   pois_bound = (int)(pois_a + 6.0 * pois_h);  // safety-bound

			while (1) 
			{
				MarsenneTwisterRNG rnd1, rnd2;
				u = rnd1.generate01();
				if (u == 0) continue;                           // avoid division by 0
				x = pois_a + pois_h * (rnd2.generate01() - 0.5) / u;
				if (x < 0 || x >= pois_bound) continue;         // reject if outside valid range
				k = (int)(x);
				lf = k * pois_g - LnFac(k) - pois_f0;
				if (lf >= u * (4.0 - u) - 3.0) break;           // quick acceptance
				if (u * (u - lf) > 1.0) continue;               // quick rejection
				if (2.0 * log(u) <= lf) break;                  // final acceptance
			}
			return k;
		}

		static int Poisson(double lambda)
		{
			if (lambda > 50) return PoissonRatioUniforms(lambda);
			double L = exp(-lambda);
			int k = 0;
			double p = 1.0;
			do
			{
				k = k + 1;
				MarsenneTwisterRNG rnd;
				p = p * rnd.generate01();
			} while (p > L);
			return k - 1;
		}
	} // end namespace math
} // end namespace semantic_mesh_segmentation
#endif /* semantic_mesh_segmentation__RANDOM_GENERATOR */
