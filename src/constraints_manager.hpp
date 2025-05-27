/*
#    Copyright (c) 2024 Juan Jose Quiroz Omana
#
#    Capybara_toolkit is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Capybara_toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with Capybara_toolkit.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, (email: juanjqogm@gmail.com)
#
# ################################################################
*/

#pragma once


#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {

constexpr bool FAIL = false;
constexpr bool SUCCESS = true;

class ConstraintsManager
{
protected:
    enum class MODE{PANIC, DO_NOT_PANIC};
    int dim_configuration_;
    MatrixXd equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd equality_constraint_vector_ = VectorXd::Zero(0);
    MatrixXd inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd inequality_constraint_vector_ = VectorXd::Zero(0);

    void _reset_equality_constraints();
    void _reset_inequality_constraints();

    static bool _check_constraint_sizes(const MatrixXd& A,
                                       const VectorXd& b,
                                       const double& optimization_vector_size,
                                       const MODE& mode = MODE::PANIC);
    static MatrixXd _resize(const MatrixXd& A, const int& rows, const int& cols);
    static MatrixXd _vstack(const MatrixXd& A, const MatrixXd& B);

public:
    ConstraintsManager() = delete;
    ConstraintsManager(const int& dim_configuration);

    void add_equality_constraint(const MatrixXd& Aeq, const VectorXd& beq);
    void add_inequality_constraint(const MatrixXd& A, const VectorXd& b);

    std::tuple<MatrixXd, VectorXd> get_equality_constraints(const bool& delete_equality_constraints = true);
    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const bool& delete_inequality_constraints = true);


};

}

