#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCholesky>
#pragma clang diagnostic pop

#include <iostream>

auto main() -> int
{
  // implement CA information filter predict step using Eigen
  Eigen::MatrixXd P{
      {{5, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0.1, 0}, {0, 0, 0, 0, 0, 1}}};
  Eigen::LDLT<Eigen::MatrixXd> ldlt(P.inverse());

  Eigen::Matrix<double, 6, 6> A{{{1, 1, 0.5, 0, 0, 0},
                                 {0, 1, 1, 0, 0, 0},
                                 {0, 0, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 0.5},
                                 {0, 0, 0, 0, 1, 1},
                                 {0, 0, 0, 0, 0, 1}}};

  Eigen::Matrix<double, 6, 2> G{{{0.5, 0}, {1, 0}, {1, 0}, {0, 0.5}, {0, 1}, {0, 1}}};

  Eigen::Matrix<double, 2, 2> Q{{{100, 0}, {0, 100}}};

  auto invA = A.inverse().eval();
  std::cout << invA << std::endl;
  auto G_ = (invA*G).eval();
  for (auto colIdx = 0; colIdx < G.cols(); ++colIdx)
  {
    auto g = G_.col(colIdx);
    std::cout << g << std::endl;
    auto Yc = ldlt.reconstructedMatrix().eval();
    std::cout << Yc << std::endl;
    auto c = 1 / ((g.transpose() * Yc * g) + 1 / Q(colIdx, colIdx));
    std::cout << c << std::endl;
    auto x = sqrt(c) * (Yc * g).eval();
    std::cout << x << std::endl;
    ldlt.rankUpdate(x, -1);
    std::cout << ldlt.reconstructedMatrix().eval() << std::endl;
  }
  auto result = (invA.transpose() * ldlt.reconstructedMatrix() * invA).eval();
  std::cout << result << std::endl;

  return 0;
}
