A generic C++ object tracking header only library for academical use to work with different filter types Extended Kalman Filter, Unscented Kalman Filter, Information Filter on the available motion models. All motion models can be configured to use a factored or normal covariance matrix and have a predictor with built-in support for the ego motion compensation. 

The factored implementations are mainly based on publications from D'Souza, Bierman, Thornton, Carlson.
* C. D'Souza and R. Zanetti, "Information Formulation of the UDU Kalman Filter," in IEEE Transactions on Aerospace and Electronic Systems, vol. 55, no. 1, pp. 493-498, Feb. 2019, doi: 10.1109/TAES.2018.2850379.
* Pourtakdoust, Seid H. “Ud Covariance Factorization For Unscented Kalman Filter Using Sequential Measurements Update,” 2007, doi:10.5281/ZENODO.1071229.
* Gerald J. Bierman, "Factorization Methods for Discrete Sequential Estimation", 1977
* Catherine L. Thornton, "Triangular Covariance Factorizations for Kalman Filtering", 1976
* Philip E. Gill, "Practical optimization", 2019, doi.org/10.1137/1.9781611975604


Key features:
* self-contained matrix library
* standard and UDU factored covariance matrix support
* filter implementations for EKF, IF, UKF
* standard motion models CV and CA with configurable covariance matrix type
* motion models support all filter models and have a built-in ego motion compensation during the prediction


Features currently in draft mode:
* class contracts in C++20 builds
* Python extension using pybind11


Non-functional requirements:
* The library is compliant to AUTOSAR-C++14 guidelines
* The minimum required C++ standard is C++17
* Error handling makes use of Rust's std::result pattern ported to C++ in https://github.com/TartanLlama/expected
* The library supports GCC and Clang compilers
* The library is comprehensivly tested and all tests are written in GoogleTest
* Test coverage is measured using lcov
* The project uses additionally cmake, clang-format, clang-tidy and doxygen
* libEigen is only used for development

