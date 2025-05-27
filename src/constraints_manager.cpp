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

#include "constraints_manager.hpp"

namespace Capybara {


ConstraintsManager::ConstraintsManager(const int &dim_configuration)
:dim_configuration_(dim_configuration)
{

}

void ConstraintsManager::_reset_equality_constraints()
{
    equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    equality_constraint_vector_ = VectorXd::Zero(0);
}

void ConstraintsManager::_reset_inequality_constraints()
{
    inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    inequality_constraint_vector_ = VectorXd::Zero(0);
}

/**
 * @brief ConstraintsManager::_check_constraint_sizes checks if the constraint set constraint A*x <= b or Ax = b
 *        have compatible sizes.
 * @param A
 * @param b
 * @param optimization_vector_size
 * @param mode
 * @return
 */
bool ConstraintsManager::_check_constraint_sizes(const MatrixXd &A, const VectorXd &b, const double &optimization_vector_size, const MODE &mode)
{
    int m = A.rows();
    int n = A.cols();
    int nb = b.size();

    if ( n != optimization_vector_size)
    {
        if (mode == MODE::PANIC )
            throw std::runtime_error(std::string("Panic with check_constraint_sizes(A, b, dim). ")
                                     + std::string("Incompatible sizes. The cols of Matrix A must have the same dimension of ")
                                     + std::string("the optimization vector, which is ") + std::to_string(optimization_vector_size)
                                     + std::string(". But the cols of A is ") + std::to_string(n));
        return Capybara::FAIL;
    }
    if (m != nb)
    {
        if (mode == MODE::PANIC )
            throw std::runtime_error(std::string("Panic with check_constraint_sizes(A, b, dim). ")
                                     + std::string("Incompatible sizes. The rows of Matrix A must have the same dimension of Vector b. ")
                                     + std::string("But you used A ")+ std::to_string(m)+ std::string("x")+ std::to_string(n)
                                     + std::string(" and b ")+ std::to_string(nb) + std::string("x1"));
        return Capybara::FAIL;
    }

    return Capybara::SUCCESS;
}

/**
 * @brief ConstraintsManager::_resize resizes a matrix A to a larger matrix of size (rows x cols) that contains
 *               the matrix A. The additional elements are zeros.
 * @param A
 * @param rows
 * @param cols
 * @return  [A 0
 *           0 0]
 */
MatrixXd ConstraintsManager::_resize(const MatrixXd &A, const int &rows, const int &cols)
{
    MatrixXd aux = MatrixXd::Zero(rows, cols);
    int m = A.rows();
    int n = A.cols();

    if (m > rows)
    {
        throw std::runtime_error(std::string("The rows you used is smaller than the rows of the Matrix. ")
                                 +std::string("Incompatible rows for resize. Matrix A has ")
                                 +std::to_string(m)+ std::string(" rows. But you used ")
                                 +std::to_string(rows));
    }
    if (n > cols)
    {
        throw std::runtime_error(std::string("The cols you used is smaller than the cols of the Matrix. ")
                                 +std::string("Incompatible cols for resize. Matrix A has ")
                                 +std::to_string(n)+ std::string(" cols. But you used ")
                                 +std::to_string(cols));
    }

    aux.block(0,0, m, n) = A;
    return aux;
}

MatrixXd ConstraintsManager::_vstack(const MatrixXd &A, const MatrixXd &B)
{

    int m_A = A.rows();
    int m_B = B.rows();
    int n_A = A.cols();
    int n_B = B.cols();
    if (n_A != n_B)
    {
            throw std::runtime_error(std::string("Incompatible sizes. The cols of Matrix A and B must have the same dimensions. ")
                                     + std::string("But A is ")+ std::to_string(A.rows())+ std::string("x")+ std::to_string(n_A)
                                     + std::string(" and B is ")+ std::to_string(B.rows()) + std::string("x")+ std::to_string(n_B));
    }
    MatrixXd C = MatrixXd::Zero(m_A + m_B, n_A);
    C.block(0,0, m_A, n_A) = A;
    C.block(m_A, 0, m_B, n_B) = B;
    return C;
}

/**
 * @brief ConstraintsManager::add_equality_constraint adds the equality constraint Aeq*x = beq
 *        to the set of equality constraints.
 *
 * @param Aeq
 * @param beq
 */
void ConstraintsManager::add_equality_constraint(const MatrixXd &Aeq, const VectorXd &beq)
{
    _check_constraint_sizes(Aeq,beq, dim_configuration_);

    if (equality_constraint_matrix_.size() == 0)
    {
        equality_constraint_matrix_ = _resize(Aeq, Aeq.rows(), dim_configuration_);
        equality_constraint_vector_ = beq;
    }else
    {
        equality_constraint_matrix_ = _vstack(equality_constraint_matrix_, Aeq);
        equality_constraint_vector_ = _vstack(equality_constraint_vector_, beq);
    }

}

/**
 * @brief ConstraintsManager::add_inequality_constraint adds the inequality constraint A*x <= b
 *        to the set of inequality constraints.
 *
 * @param A
 * @param b
 */
void ConstraintsManager::add_inequality_constraint(const MatrixXd &A, const VectorXd &b)
{

    _check_constraint_sizes(A,b, dim_configuration_);

    if (inequality_constraint_matrix_.size() == 0)
    {
        inequality_constraint_matrix_ = _resize(A, A.rows(), dim_configuration_);
        inequality_constraint_vector_ = b;
    }else
    {
        inequality_constraint_matrix_ = _vstack(inequality_constraint_matrix_, A);
        inequality_constraint_vector_ = _vstack(inequality_constraint_vector_, b);
    }

}

/**
 * @brief ConstraintsManager::get_equality_constraints returns the set of equality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x = b.
 *
 * @param delete_equality_constraints Flag used to delete all the equality constraints stored. (Default)
 * @return {Aeq, beq}
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_equality_constraints(const bool &delete_equality_constraints)
{
    auto Aeq = equality_constraint_matrix_;
    auto beq = equality_constraint_vector_;
    if (delete_equality_constraints)
        _reset_equality_constraints();
    return {Aeq, beq};
}


/**
 * @brief ConstraintsManager::get_inequality_constraints returns the set of inequality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x <= b
 *
 * @param delete_inequality_constraints Flag used to delete all the inequality constraints stored. (Default)
 * @return
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_inequality_constraints(const bool &delete_inequality_constraints)
{
    auto A = inequality_constraint_matrix_;
    auto b = inequality_constraint_vector_;
    if (delete_inequality_constraints)
        _reset_inequality_constraints();
    return {A, b};
}





}
