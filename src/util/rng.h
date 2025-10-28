/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>

namespace rng
{

/**
 */
struct RNG
{
    RNG() = default;
    virtual ~RNG() = default;

    // returns double in [0,1)
    virtual double operator ()() = 0;
};

struct XorShift64 : public RNG
{
    XorShift64(uint64_t seed = 123456789) : state(seed) {}
    virtual ~XorShift64() = default;

    uint64_t randomNumber() 
    {
        uint64_t x = state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        state = x;
        return x;
    }

    // returns double in [0,1)
    double operator ()() override 
    { 
        return (randomNumber() >> 11) * Norm; 
    }

    uint64_t state;

    static const double Norm;
};

/**
 */
struct XorShift128Plus : public RNG
{
    XorShift128Plus(uint64_t seed = 123456789) 
    {
        s[0] = seed;
        s[1] = seed ^ 0xdeadbeefcafebabeULL;
    }

    virtual ~XorShift128Plus() = default;

    uint64_t randomNumber() 
    {
        uint64_t x = s[0];
        uint64_t y = s[1];
        s[0] = y;
        x ^= x << 23;
        s[1] = x ^ y ^ (x >> 17) ^ (y >> 26);
        return s[1] + y;
    }

    // returns double in [0,1)
    double operator ()() override 
    {
        return (randomNumber() >> 11) * Norm;
    }

    uint64_t s[2];

    static const double Norm;
};

/**
 */
struct PCG32 : public RNG
{
    PCG32(uint64_t seed = 42, uint64_t seq = 54) : state(0), inc((seq << 1) | 1) 
    {
        randomNumber();
        state += seed;
        randomNumber();
    }

    virtual ~PCG32() = default;

    uint32_t randomNumber() 
    {
        uint64_t oldstate = state;
        state = oldstate * 6364136223846793005ULL + inc;
        uint32_t xorshifted = ((oldstate >> 18) ^ oldstate) >> 27;
        uint32_t rot = oldstate >> 59;
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }

    double operator ()() override 
    { 
        return randomNumber() * Norm; 
    }

    uint64_t state;
    uint64_t inc;

    static const double Norm;
};

/**
 */
class DiscreteSampler
{
public:
    static size_t sample(RNG& rng,
                         const double* distribution,
                         size_t n,
                         const std::vector<char>* mask = nullptr);
};

} // namespace rng
