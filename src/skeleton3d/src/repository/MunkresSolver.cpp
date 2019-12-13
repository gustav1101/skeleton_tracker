#include "MunkresSolver.hpp"
#include "../external/munkres-cpp/src/munkres-cpp/munkres.h"
#include "../external/munkres-cpp/src/munkres-cpp/adapters/matrix_std_2d_vector.h"
#include "../external/munkres-cpp/src/munkres-cpp/utils.h"

template<class T> using vector = std::vector<T>;
using Matrix = munkres_cpp::matrix_std_2d_vector<double>;

namespace MunkresSolver
{
    bool check_validity(Matrix& distance_matrix);
    Matrix convert_to_munkres_matrix(
        vector<vector<double>>& distance_matrix);
    void solve(Matrix& distance_matrix);
    void copy_results_to_vector_matrix(
        Matrix& source_matrix, vector<vector<double>>& vector_matrix);
    bool is_actually_a_matrix(vector<vector<double>>& vector_matrix);
}

void MunkresSolver::solve_munkres(vector<vector<double>>& distance_matrix)
{
    if(!is_actually_a_matrix(distance_matrix))
    {
        throw std::invalid_argument("Distance matrix has unmatching number of columns");
    }
    Matrix munkres_matrix = convert_to_munkres_matrix(distance_matrix);
    if (!check_validity(munkres_matrix))
    {
        throw std::invalid_argument("Distance matrix holds negative values");
    }
    solve(munkres_matrix);
    copy_results_to_vector_matrix(munkres_matrix, distance_matrix);
}

bool MunkresSolver::is_actually_a_matrix(vector<vector<double>> &vector_matrix)
{
    int first_row_size = vector_matrix.size();
    if( first_row_size == 0)
    {
        return false;
    }
    if( first_row_size == 1)
    {
        return true;
    }
    int number_of_columns = vector_matrix.at(0).size();
    for(int row = 1; row < vector_matrix.size(); row++)
    {
        if( number_of_columns != vector_matrix.at(row).size() )
        {
            return false;
        }
    }
    return true;
}

bool MunkresSolver::check_validity(Matrix& distance_matrix)
{
    return munkres_cpp::is_data_valid(distance_matrix);
}

Matrix MunkresSolver::convert_to_munkres_matrix(vector<vector<double>>& distance_matrix)
{
    return Matrix(distance_matrix);
}

void MunkresSolver::solve(Matrix &distance_matrix)
{
    // Yes, I know. Appareantly that's how you solve it.
    munkres_cpp::Munkres<double> solver(distance_matrix);
}

void MunkresSolver::copy_results_to_vector_matrix(Matrix& source_matrix, vector<vector<double>>& vector_matrix)
{
    for(int row = 0; row < vector_matrix.size(); row++)
    {
        for(int column = 0; column < vector_matrix.at(0).size(); column++)
        {
            vector_matrix.at(row).at(column) = source_matrix(row, column);
        }
    }
}
