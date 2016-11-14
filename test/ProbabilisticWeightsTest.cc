#include <limits>
#include <vector>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include "point_cloud_registration/probabilistic_weights.hpp"

using point_cloud_registration::ProbabilisticWeights;

std::vector<double> squaredErrors()
{
    /**
    Squared Errors Matrix = | 1 0 1 1 |
                            |1 4 9 16 |
    **/
    std::vector<double> squared_errors = {1, 1, 1, 1, 4, 9, 16};
    return squared_errors;
}
Eigen::SparseMatrix<int, Eigen::RowMajor> dataAssociation()
{
    Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(2,4);
    std::vector<Eigen::Triplet<int>> tripletList;
    tripletList.push_back(Eigen::Triplet<int>(0, 0, 1));
    tripletList.push_back(Eigen::Triplet<int>(0, 2, 1));
    tripletList.push_back(Eigen::Triplet<int>(0, 3, 1));
    tripletList.push_back(Eigen::Triplet<int>(1, 0, 1));
    tripletList.push_back(Eigen::Triplet<int>(1, 1, 1));
    tripletList.push_back(Eigen::Triplet<int>(1, 2, 1));
    tripletList.push_back(Eigen::Triplet<int>(1, 3, 1));
    data_association.setFromTriplets(tripletList.begin(), tripletList.end());
    data_association.makeCompressed();
    return data_association;
}

TEST(UpdateWeightsTestSuite, tCallbackTest)
{
    auto data_association = dataAssociation();
    std::vector<double> squared_errors = squaredErrors();
    ProbabilisticWeights weightUpdater(5, 1, 4);
    auto weights = weightUpdater.updateWeights(data_association, squared_errors);
    Eigen::MatrixXd expected_weights(2, 4);
    expected_weights << 1.0 / 3, 0, 1.0 / 3, 1.0 / 3, 0.7151351, 0.1412613,
                     0.0241258, 0.0047656;
    for (std::size_t i = 0; i < expected_weights.rows(); i++)
    {
        for (std::size_t j = 0; j < expected_weights.cols(); j++)
        {
            EXPECT_NEAR(expected_weights(i, j), weights.coeffRef(i, j), 1e-6);
        }
    }
}

TEST(UpdateWeightsTestSuite, gaussianCallbackTest)
{
    auto data_association = dataAssociation();
    std::vector<double> squared_errors =  squaredErrors();
    ProbabilisticWeights weightUpdater(std::numeric_limits<double>::infinity(), 1,
                                       4);
    auto weights = weightUpdater.updateWeights(data_association, squared_errors);
    Eigen::MatrixXd expected_weights(2, 4);
    expected_weights << 1.0 / 3, 0, 1.0 / 3, 1.0 / 3, 0.805153702921689,
                     0.179654074677018, 0.0147469044726408, 0.000445317928652638;
    for (std::size_t i = 0; i < expected_weights.rows(); i++)
    {
        for (std::size_t j = 0; j < expected_weights.cols(); j++)
        {
            EXPECT_NEAR(expected_weights(i, j), weights.coeffRef(i, j), 1e-6);
        }
    }
}
