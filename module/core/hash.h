#ifndef SCAN3D_CORE_HASH_H
#define SCAN3D_CORE_HASH_H


#include <cstdint>
#include <functional>


namespace scan3d
{
    /**
     * Liangliang: std::size_t has 64 bits on most systems, but 32 bits on 32-bit Windows. To make the same code
     * robustly run on both 32-bit and 64-bit systems, I use 64-bit integer for hash values.
     */

#if 0
    /**
     * The hash combine function copied from boost with the integer type std::size_t changed to uint64_t.
     * I found an example that can fail this function (tested on macOS Catalina Version 10.15.4):
     *      std::vector<float> a = {16, 0}; // hash: 240982999006
     *      std::vector<float> b = {4, 12}; // hash: 240982999006
     *      std::cout << "a: " << hash_range(a.begin(), a.end()) << std::endl;
     *      std::cout << "b: " << hash_range(b.begin(), b.end()) << std::endl;
     * That is why I need a function for 64-bit integers.
     */
    template<class T>
    inline void hash_combine(uint64_t &seed, T const& value) {
        std::hash<T> hasher;
        seed ^= hasher(value) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }

#else
    /**
     * The 64-bit hash combine algorithm from CityHash
     * https://github.com/google/cityhash/blob/master/src/city.h
     */
    template<class T>
    inline void hash_combine(uint64_t &seed, T const& value) {
        std::hash<T> hasher;
        uint64_t a = (hasher(value) ^ seed) * 0x9ddfea08eb382d69ULL;
        a ^= (a >> 47);
        uint64_t b = (seed ^ a) * 0x9ddfea08eb382d69ULL;
        b ^= (b >> 47);
        seed = b * 0x9ddfea08eb382d69ULL;
    }
#endif

    template <typename FT>
    uint64_t hash(const Vec<2, FT>& value) {
        uint64_t seed = 0;
        hash_combine(seed, value.x);
        hash_combine(seed, value.y);
        return seed;
    }

    template <typename FT>
    uint64_t hash(const Vec<3, FT>& value) {
        uint64_t seed = 0;
        hash_combine(seed, value.x);
        hash_combine(seed, value.y);
        hash_combine(seed, value.z);
        return seed;
    }


    template <int DIM, typename FT> inline
    uint64_t hash(const Vec<DIM, FT>& value) {
        uint64_t seed = 0;
        for (uint64_t i=0; i<DIM; ++i)
            hash_combine(seed, value[i]);
        return seed;
    }


    template<class It>
    inline uint64_t hash_range(It first, It last) {
        uint64_t seed = 0;
        for (; first != last; ++first) {
            hash_combine(seed, *first);
        }
        return seed;
    }

    template<class It>
    inline void hash_range(uint64_t &seed, It first, It last) {
        for (; first != last; ++first) {
            hash_combine(seed, *first);
        }
    }

} // namespace scan3d

#endif  // SCAN3D_CORE_HASH_H