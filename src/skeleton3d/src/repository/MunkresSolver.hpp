#ifndef MUNKRESSOLVER_HPP
#define MUNKRESSOLVER_HPP

#include <vector>

namespace MunkresSolver
{
    template<class T> using vector = std::vector<T>;
    void solve_munkres(vector<vector<double>>& distance_matrix);
}

#endif
