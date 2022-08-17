#include<iostream>
#include <vector>
#include<iomanip>
#include"math.h"
#include<bits/stdc++.h>
#include <fstream>
#ifndef __CUBIC_SPLINE__
#define __CUBIC_SPLINE__



class CubicSpline{
public:
    // CubicSpline();
    // ~CubicSpline();
    void initnew(std::vector<std::vector<double>> setxy);
    std::vector<double> calc_row_fn(int index, int position, std::vector<std::vector<double>> setxy);
    std::vector<double> calc_row_fn_dot(int index, std::vector<std::vector<double>> setxy);
    std::vector<double> calc_row_fn_dot_dot(int index, std::vector<std::vector<double>> setxy);
    std::vector<std::vector<double>> natural_spline( std::vector<std::vector<double>> setxy);
    std::vector<std::vector<double>> get_main_matrix(std::vector<std::vector<double>> setxy);
    std::vector<std::vector<double>> gaussian_elimination(std::vector<std::vector<double>> a_matrix);
    std::vector<std::vector<double>> spline_sampling(std::vector<std::vector<double>> xy_set, std::vector<std::vector<double>> coeff_sets, double sample_fraction);
    void write_out_csv(std::string address, std::vector<std::vector<double>> setxy);
    std::vector<std::vector<double>> calc_all(std::vector<std::vector<double>> setxy, double sample_fraction);
    std::vector<double> calc_s_cumilative_sum( std::vector<std::vector<double>> setxy);

private:
    std::vector<std::vector<double>> input_setxy;   // input set x and y has n+1 points, input_set[0] = set of x  ,input_set[1] = set of y
    int n_fn  ;            // number of functions  
    std::vector<std::vector<double>> main_matrix;   // matrix to solve 4n coefficient size (n, n+1)
    std::vector<double> set_co_a;                   // coefficient of following function f(x) = y = a + b*x + c*x**2 + d*x**3 
    std::vector<double> set_co_b;
    std::vector<double> set_co_c;
    std::vector<double> set_co_d;
    std::vector<std::vector<double>> output_setxy;

};

class CubicSpline2D:public CubicSpline{
public:
    void initnew_2d(std::vector<std::vector<double>> setxy);
    std::vector<double> calc_s_cumilative_sum( std::vector<std::vector<double>> setxy);
private:
    std::vector<double> s_cumilative_sum;

};

#endif