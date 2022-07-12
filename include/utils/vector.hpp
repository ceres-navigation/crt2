#ifndef __VECTOR_H_
#define __VECTOR_H_

#include <array>
#include <cmath>

// TODO: This was borrowed from another library...  implement my own at somepoint!
template <typename, size_t> struct Vec;

/// Helper class to set the elements of a vector.
template <size_t I, typename Scalar, size_t N>
struct VecSetter {
    template <typename... Args>
    static void set(Vec<Scalar, N>& v, Scalar s, Args... args) {
        v[I] = s;
        VecSetter<I + 1, Scalar, N>::set(v, args...);
    }
};

template <typename Scalar, size_t N>
struct VecSetter<N, Scalar, N> {
    static void set(Vec<Scalar, N>&) {}
};

/// An N-dimensional vector class.
template <typename Scalar, size_t N>
struct Vec {
    Scalar values[N];

    Vec() = default;
    explicit Vec(Scalar s) { std::fill(values, values + N, s); }

    template <size_t M, std::enable_if_t<(M > N), int> = 0>
    explicit Vec(const Vec<Scalar, M>& other) {
        std::copy(other.values, other.values + N, values);
    }

    template<typename T>
    explicit Vec(const std::array<T, N>& arr) {
        for (size_t i = 0; i < N; ++i)
            values[i] = arr[i];
    }

    template <typename... Args>
    Vec(Scalar first, Scalar second, Args... args) {
        set(first, second, args...);
    }

    template <typename F, std::enable_if_t<std::is_invocable<F, size_t>::value, int> = 0>
    Vec(F f) {
        for (size_t i = 0; i < N; ++i)
            values[i] = f(i);
    }

    template <typename... Args>
    void set(Args... args) {
        VecSetter<0, Scalar, N>::set(*this, Scalar(args)...);
    }

    Vec operator - () const {
        return Vec([this] (size_t i) { return -values[i]; });
    }

    Vec inverse() const {
        return Vec([this] (size_t i) { return Scalar(1) / values[i]; });
    }

    Vec safe_inverse() const {
        static constexpr auto threshold = std::numeric_limits<Scalar>::epsilon();
        return Vec([&] (size_t i) {
            return Scalar(1) / (std::fabs(values[i]) < threshold ? std::copysign(threshold, values[i]) : values[i]);
        });
    }

    Vec& operator += (const Vec& other) {
        return *this = *this + other;
    }

    Vec& operator -= (const Vec& other) {
        return *this = *this - other;
    }

    Vec& operator *= (const Vec& other) {
        return *this = *this * other;
    }

    Scalar& operator [] (size_t i) { return values[i]; }
    Scalar  operator [] (size_t i) const { return values[i]; }
};

template <typename Scalar, size_t N>
inline Vec<Scalar, N> operator + (const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    return Vec<Scalar, N>([=] (size_t i) { return a[i] + b[i]; });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> operator - (const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    return Vec<Scalar, N>([=] (size_t i) { return a[i] - b[i]; });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> operator * (const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    return Vec<Scalar, N>([=] (size_t i) { return a[i] * b[i]; });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> min(const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    return Vec<Scalar, N>([=] (size_t i) { return std::min(a[i], b[i]); });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> max(const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    return Vec<Scalar, N>([=] (size_t i) { return std::max(a[i], b[i]); });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> operator * (const Vec<Scalar, N>& a, Scalar s) {
    return Vec<Scalar, N>([=] (size_t i) { return a[i] * s; });
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> operator * (Scalar s, const Vec<Scalar, N>& b) {
    return b * s;
}

template <typename Scalar, size_t N>
inline Scalar dot(const Vec<Scalar, N>& a, const Vec<Scalar, N>& b) {
    Scalar sum = a[0] * b[0];
    for (size_t i = 1; i < N; ++i)
        sum += a[i] * b[i];
    return sum;
}

template <typename Scalar, size_t N>
inline Scalar length(const Vec<Scalar, N>& v) {
    return std::sqrt(dot(v, v));
}

template <typename Scalar, size_t N>
inline Vec<Scalar, N> normalize(const Vec<Scalar, N>& v) {
    auto inv = Scalar(1) / length(v);
    return v * inv;
}

template <typename Scalar>
inline Vec<Scalar, 3> cross(const Vec<Scalar, 3>& a, const Vec<Scalar, 3>& b) {
    return Vec<Scalar, 3>([=] (size_t i) {
        size_t j = (i + 1) % 3;
        size_t k = (i + 2) % 3;
        return a[j] * b[k] - a[k] * b[j];
    });
}

template <typename Scalar>
inline Vec<Scalar, 3> translate(const Vec<Scalar, 3>& a, const Vec<Scalar, 3>& b){
    return Vec<Scalar, 3>(a[0]+b[0], a[1]+b[1], a[2]+b[2]);
}

template <typename Scalar>
using Vector3 = Vec<Scalar, 3>;

template <typename Scalar>
using Vector2 = Vec<Scalar, 2>;

#endif