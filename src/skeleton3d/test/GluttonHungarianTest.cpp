#include <catch_ros/catch.hpp>
#include "../src/repository/MunkresSolver.hpp"

template<class T> using vector = std::vector<T>;

SCENARIO("Using the glutton Hungarian framework")
{
    GIVEN("A small symmetrical distance matrix")
    {
        vector<vector<double>> distance_matrix =
            {
                {10.8, 12.5, 15.0},
                {15.0, 13.5, 17.5},
                {12.2, 14.8, 25.0}
            };
        THEN("The matrix is solved correctly")
        {
            MunkresSolver::solve_munkres(distance_matrix);
            for(int row=0; row < 3; row++)
            {
                for(int column = 0; column < 3; column++)
                {
                    if(
                        (row == 0 && column == 2) ||
                        (row == 1 && column == 1) ||
                        (row == 2 && column == 0)
                        )
                    {
                        REQUIRE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                    else
                    {
                        REQUIRE_FALSE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                }
            }
        }
    }
    GIVEN("A large symmetrical matrix")
    {
        vector<vector<double>> distance_matrix =
            {
                {2.0, 6.0, 6.8, 9.7, 2.9, 8.6},
                {3.0, 1.0, 2.4, 7.4, 4.1, 2.6},
                {6.6, 4.0, 2.9, 4.9, 7.1, 6.3},
                {6.2, 8.2, 8.7, 1.0, 4.4, 4.3},
                {9.0, 2.0, 1.8, 3.7, 2.3, 6.2},
                {5.0, 2.7, 4.3, 3.3, 2.9, 7.2}
            };
        THEN("The matrix is solved correctly")
        {
            MunkresSolver::solve_munkres(distance_matrix);
            for(int row = 0; row < 6; row++)
            {
                for(int column = 0; column < 6; column++)
                {
                    if(
                        (row == 0 && column == 0) ||
                        (row == 1 && column == 5) ||
                        (row == 2 && column == 2) ||
                        (row == 3 && column == 3) ||
                        (row == 4 && column == 1) ||
                        (row == 5 && column == 4)
                        )
                    {
                        REQUIRE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                    else
                    {
                        REQUIRE_FALSE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                }
            }   
        }
    }
    GIVEN("A large assymetrical matrix (more columns than rows)")
    {
        vector<vector<double>> distance_matrix =
            {
                {6.5, 2.1, 8.3, 2.3, 7.9, 2.7, 2.1},
                {7.5, 9.9, 2.0, 9.5, 4.0, 9.5, 5.8},
                {5.6, 0.1, 0.4, 3.5, 7.4, 8.6, 2.0},
                {2.4, 2.6, 1.4, 2.3, 2.8, 6.8, 5.3}
            };
        THEN("The matrix is still solved correctly")
        {
            MunkresSolver::solve_munkres(distance_matrix);
            for(int row = 0; row < 4; row++)
            {
                for(int column = 0; column < 7; column++)
                {
                    if(
                        (row == 0 && column == 6) ||
                        (row == 1 && column == 2) ||
                        (row == 2 && column == 1) ||
                        (row == 3 && column == 3)
                        )
                    {
                        REQUIRE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                    else
                    {
                        REQUIRE_FALSE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                }
            } 
        }
    }
    GIVEN("A large assymetrical matrix (more rows than columns)")
    {
        vector<vector<double>> distance_matrix =
            {
                {3.4, 4.9, 8.6},
                {7.7, 0.4, 0.4},
                {8.9, 3.9, 3.6},
                {1.5, 5.7, 8.5},
                {3.6, 0.2, 3.5},
                {3.7, 2.9, 1.7},
                {6.5, 3.3, 8.7}
            };
        THEN("The matrix is still solved correctly")
        {
            MunkresSolver::solve_munkres(distance_matrix);
            for(int row = 0; row < 7; row++)
            {
                for(int column = 0; column < 3; column++)
                {
                    if(
                        (row == 1 && column == 2) ||
                        (row == 3 && column == 0) ||
                        (row == 4 && column == 1)
                        )
                    {
                        REQUIRE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                    else
                    {
                        REQUIRE_FALSE(distance_matrix.at(row).at(column) == Approx(0.0));
                    }
                }
            }
        }
    }
    GIVEN("A Non-Matrix (one value is negative)")
    {
        vector<vector<double>> distance_matrix =
            {
                {3.4, 4.9, 8.6},
                {3.7, 2.9, -5.0},
                {6.5, 3.3, 8.7}
            };
        THEN("It should throw an exception")
        {
            REQUIRE_THROWS_AS(MunkresSolver::solve_munkres(distance_matrix), std::invalid_argument);
        }
    }
    GIVEN("A Non-Matrix (one row has less values)")
    {
        vector<vector<double>> distance_matrix =
            {
                {3.4, 4.9, 8.6},
                {3.7, 2.9},
                {6.5, 3.3, 8.7}
            };
        THEN("It should throw an exception")
        {
            REQUIRE_THROWS_AS(MunkresSolver::solve_munkres(distance_matrix), std::invalid_argument);
        }
    }
}
