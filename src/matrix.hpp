#include <algorithm>
#include <array>
#include <vector>

namespace linalg {

template <std::size_t N, std::size_t M, typename T = int64_t>
class Matrix {
 private:
  static constexpr std::size_t kHeight = N;
  static constexpr std::size_t kWidth = M;

 public:
  Matrix() = default;

  explicit Matrix(const std::vector<std::vector<T>>& vector) {
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        (*this)(line_idx, column_idx) = vector[line_idx][column_idx];
      }
    }
  }

  explicit Matrix(const T& value) { fill(begin(), end(), value); }

  ~Matrix() = default;

 public:
  Matrix operator+(const Matrix& other) const {
    Matrix copy(*this);
    copy += other;

    return copy;
  }
  Matrix& operator+=(const Matrix& other) {
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        (*this)(line_idx, column_idx) += other(line_idx, column_idx);
      }
    }

    return *this;
  }

  Matrix operator-(const Matrix& other) const {
    Matrix copy(*this);
    copy -= other;

    return copy;
  }
  Matrix& operator-=(const Matrix& other) {
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        (*this)(line_idx, column_idx) -= other(line_idx, column_idx);
      }
    }

    return *this;
  }

  Matrix operator*(const T& scalar) const {
    Matrix copy(*this);
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        copy(line_idx, column_idx) *= scalar;
      }
    }

    return copy;
  }

  friend Matrix<N, M, T> operator*(const T& scalar,
                                   const Matrix<N, M, T>& matrix) {
    Matrix<N, M, T> copy(matrix);
    for (std::size_t line_idx = 0; line_idx < Matrix<N, M, T>::kHeight;
         ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < Matrix<N, M, T>::kWidth;
           ++column_idx) {
        copy(line_idx, column_idx) *= scalar;
      }
    }

    return copy;
  }

  T& operator()(std::size_t line_idx, std::size_t column_idx) {
    return data_[(line_idx * kWidth) + column_idx];
  }
  const T& operator()(std::size_t line_idx, std::size_t column_idx) const {
    return data_[(line_idx * kWidth) + column_idx];
  }

  bool operator==(const Matrix& other) const {
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        if (other(line_idx, column_idx) != (*this)(line_idx, column_idx)) {
          return false;
        }
      }
    }

    return true;
  }

 public:
  Matrix<kWidth, kHeight, T> transposed() const {
    Matrix<kWidth, kHeight, T> result{};
    for (std::size_t line_idx = 0; line_idx < kHeight; ++line_idx) {
      for (std::size_t column_idx = 0; column_idx < kWidth; ++column_idx) {
        result(column_idx, line_idx) = (*this)(line_idx, column_idx);
      }
    }

    return result;
  }

 private:
  T* begin() { return data_.begin(); }
  const T* begin() const { return data_.begin(); }

  T* end() { return data_.end(); }
  const T* end() const { return data_.end(); }

  void fill(T* const kBegin, T* const kEnd, const T& value) {
    std::fill(kBegin, kEnd, value);
  }

 private:
  std::array<T, N * M> data_{};
};

template <std::size_t N, std::size_t M, std::size_t K, typename T = int64_t>
Matrix<N, K, T> operator*(const Matrix<N, M, T>& lhs,
                          const Matrix<M, K, T>& rhs) {
  Matrix<K, M, T> rhs_transposed{};
  for (std::size_t i = 0; i < M; ++i) {
    for (std::size_t j = 0; j < K; ++j) {
      rhs_transposed(j, i) = rhs(i, j);
    }
  }

  Matrix<N, K, T> result{};
  for (std::size_t line_idx = 0; line_idx < N; ++line_idx) {
    for (std::size_t column_idx = 0; column_idx < K; ++column_idx) {
      T result_element = T{};
      for (std::size_t sum_iterator = 0; sum_iterator < M; ++sum_iterator) {
        result_element += lhs(line_idx, sum_iterator)
                          * rhs_transposed(column_idx, sum_iterator);
      }
      result(line_idx, column_idx) = result_element;
    }
  }

  return result;
}

}  // namespace linalg
