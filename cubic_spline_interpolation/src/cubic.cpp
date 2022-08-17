#include <iostream>
#include <cubic.h>

void CubicSpline::initnew(std::vector<std::vector<double>> setxy){


    CubicSpline::n_fn = setxy[0].size() - 1;
    CubicSpline::input_setxy = setxy;
}

std::vector<double> CubicSpline::calc_row_fn(int index, int position, std::vector<std::vector<double>> setxy){


    int n_ = setxy[0].size() - 1;
    std::vector<double> row_n ;
    for(int z = 0; z < 4*position ; z ++){
        row_n.push_back(0.0);
    }
    row_n.push_back(1);
    row_n.push_back(pow(setxy[0][index],1));
    row_n.push_back(pow(setxy[0][index],2));
    row_n.push_back(pow(setxy[0][index],3));
    for(int r = 0; r < (4*n_ - 4*(position+1)); r+=1){
        row_n.push_back(0);
    }
    row_n.push_back(setxy[1][index]);
    // std::cout << std::endl;

    return row_n;

}

std::vector<double> CubicSpline::calc_row_fn_dot(int index, std::vector<std::vector<double>> setxy){
    int n_ = setxy[0].size() - 1;
    std::vector<double> row_n ;

    for(int z = 0; z < 4*(index-1) ; z ++){
        row_n.push_back(0.0);
    }
    row_n.push_back(0);
    row_n.push_back(1);
    row_n.push_back(2*pow(setxy[0][index],1));
    row_n.push_back(3*pow(setxy[0][index],2));
    row_n.push_back(0);
    row_n.push_back(-1);
    row_n.push_back(-2*pow(setxy[0][index],1));
    row_n.push_back(-3*pow(setxy[0][index],2));
    for(int r = 0; r < (4*n_ - 4*(index+1)); r+=1){
        row_n.push_back(0);
    }
    row_n.push_back(0);
    // std::cout << std::endl;
    
    return row_n;
}

std::vector<double> CubicSpline::calc_row_fn_dot_dot(int index, std::vector<std::vector<double>> setxy){
    int n_ = setxy[0].size() - 1;
    std::vector<double> row_n ;

    for(int z = 0; z < 4*(index-1) ; z ++){
        row_n.push_back(0.0);
    }
    row_n.push_back(0);
    row_n.push_back(0);
    row_n.push_back(2);
    row_n.push_back(6*pow(setxy[0][index],1));
    row_n.push_back(0);
    row_n.push_back(0);
    row_n.push_back(-2);
    row_n.push_back(-6*pow(setxy[0][index],1));
    for(int r = 0; r < (4*n_ - 4*(index+1)); r+=1){
        row_n.push_back(0);
    }
    row_n.push_back(0);
    // std::cout << std::endl;
    
    return row_n;
}

std::vector<std::vector<double>> CubicSpline::natural_spline( std::vector<std::vector<double>> setxy){
    int n_ = setxy[0].size() - 1;
    std::vector<std::vector<double>> row_2n ;
    std::vector<double> row_n_1 ;
    std::vector<double> row_n_2 ;

    row_n_1.push_back(0);
    row_n_1.push_back(0);
    row_n_1.push_back(2);
    row_n_1.push_back(6*pow(setxy[0][0],1));
    for(int r = 0; r < (4*(n_ -1 )); r+=1){
        row_n_1.push_back(0);
    }
    row_n_1.push_back(0);

    for(int z = 0; z < 4*(n_-1) ; z ++){
        row_n_2.push_back(0.0);
    }
    row_n_2.push_back(0);
    row_n_2.push_back(0);
    row_n_2.push_back(2);
    row_n_2.push_back(6*pow(setxy[0][n_fn],1));

    row_n_2.push_back(0);

    // std::cout << std::endl;
    row_2n.push_back(row_n_1);
    row_2n.push_back(row_n_2);
    
    return row_2n;
}

std::vector<std::vector<double>> CubicSpline::get_main_matrix(std::vector<std::vector<double>> setxy){
    // first round 2n rows
    int n_ = CubicSpline::n_fn;
    // std::vector<double> row_n ;
    for(int i = 0; i < n_; i++){
        std::vector<double> temp_1;
        std::vector<double> temp_2;
        temp_1 = CubicSpline::calc_row_fn(i,i, setxy);
        temp_2 = CubicSpline::calc_row_fn(i+1,i, setxy);
        CubicSpline::main_matrix.push_back(temp_1);
        CubicSpline::main_matrix.push_back(temp_2);

    }
    for(int i =1;i<n_;i++){
        std::vector<double> temp_;
        temp_ = CubicSpline::calc_row_fn_dot(i, setxy);
        CubicSpline::main_matrix.push_back(temp_);
    }
    for(int i =1;i<n_;i++){
        std::vector<double> temp_;
        temp_ = CubicSpline::calc_row_fn_dot_dot(i, setxy);
        CubicSpline::main_matrix.push_back(temp_);
    }
    std::vector<std::vector<double>> last_constraint_temp;
    last_constraint_temp = CubicSpline::natural_spline( setxy);
    CubicSpline::main_matrix.push_back(last_constraint_temp[0]);
    CubicSpline::main_matrix.push_back(last_constraint_temp[1]);

    return main_matrix;
}
std::vector<std::vector<double>> CubicSpline::gaussian_elimination(std::vector<std::vector<double>> a_matrix){

    int n = a_matrix.size();
    int i,j,k;
    std::vector<double> x;
    std::vector<std::vector<double>> dummy;
    for(int i =0; i< n; i++){
        x.push_back(0);
    }

    for (i=0;i<n;i++)                    //Pivotisation
        for (k=i+1;k<n;k++)
            if (abs(a_matrix[i][i])<abs(a_matrix[k][i]))
                for (j=0;j<=n;j++)
                {
                    double temp=a_matrix[i][j];
                    a_matrix[i][j]=a_matrix[k][j];
                    a_matrix[k][j]=temp;
                }

    for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n;k++)
            {
                double t=a_matrix[k][i]/a_matrix[i][i];
                for (j=0;j<=n;j++)
                    a_matrix[k][j]=a_matrix[k][j]-t*a_matrix[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
     
    for (i=n-1;i>=0;i--)                //back-substitution
    {                       
        x[i]=a_matrix[i][n];                
        for (j=i+1;j<n;j++)
            if (j!=i)           
                x[i]=x[i]-a_matrix[i][j]*x[j];
        x[i]=x[i]/a_matrix[i][i];           
    }
 
    std::vector<double> set_a;
    std::vector<double> set_b;
    std::vector<double> set_c;
    std::vector<double> set_d;
    std::vector<std::vector<double>> set_coef_;
    // std::cout << "value set a " ;
    for( int i = 0; i < n/4; i++){
        set_a.push_back(x[i*4+0]);
        set_b.push_back(x[i*4+1]);
        set_c.push_back(x[i*4+2]);
        set_d.push_back(x[i*4+3]);
        // std::cout << x[i*4+0] << " " ;
    }
    // std::cout << std::endl;
    set_coef_.push_back(set_a);
    set_coef_.push_back(set_b);
    set_coef_.push_back(set_c);
    set_coef_.push_back(set_d);

    return set_coef_;

}

std::vector<std::vector<double>> CubicSpline::spline_sampling(std::vector<std::vector<double>> xy_set, 
                            std::vector<std::vector<double>> coeff_sets, double sample_fraction){
    std::vector<double> output_x_set ;
    std::vector<double> output_y_set ;
    std::vector<std::vector<double>> output_xy_set;
    for(double i = xy_set[0][0] + 1e-10; i < xy_set[0][xy_set[0].size()-1]; i+= sample_fraction){
        int bisect = std::lower_bound(xy_set[0].begin(), xy_set[0].end(), i)- xy_set[0].begin() -1 ;
        // std::cout << " max x " << xy_set[0].size() << std::endl;
        output_x_set.push_back(i);
        // std::cout << "bisect " <<bisect << std::endl;
        output_y_set.push_back(  coeff_sets[0][bisect] +
                                +coeff_sets[1][bisect] * pow(i,1)
                                +coeff_sets[2][bisect] * pow(i,2)
                                +coeff_sets[3][bisect] * pow(i,3)  );
    }
    output_x_set.push_back(xy_set[0][xy_set[0].size()-1]);
    output_y_set.push_back(xy_set[1][xy_set[0].size()-1]);

    output_xy_set.push_back(output_x_set);
    output_xy_set.push_back(output_y_set);

    return output_xy_set;
}
std::vector<std::vector<double>> CubicSpline::calc_all(std::vector<std::vector<double>> setxy,double sample_fraction){
    initnew(setxy);
    std::vector<std::vector<double>> main_matrix = get_main_matrix(setxy);
    
    std::vector<std::vector<double>> output_coef_set = gaussian_elimination(main_matrix);

    std::vector<std::vector<double>> output_xy_set;
    output_xy_set = spline_sampling(setxy, output_coef_set, sample_fraction);
    return output_xy_set;
}

void CubicSpline::write_out_csv(std::string address, std::vector<std::vector<double>> set_xy){

    // std::ofstream inmyFile("../src/inputcubic.csv");
    std::ofstream inmyFile(address);
    for (unsigned int i = 0; i < static_cast<unsigned int>(set_xy[0].size()); i++){
        inmyFile << set_xy[0][i] << ", " << set_xy[1][i] << std::endl;
    }
    inmyFile.close();
}
void CubicSpline2D::initnew_2d(std::vector<std::vector<double>> setxy){

    std::cout << std::endl;
}

std::vector<double> CubicSpline::calc_s_cumilative_sum( std::vector<std::vector<double>> setxy){
    int n_fn = setxy[0].size() - 1;
    std::vector<double> s_cum_sum;
    s_cum_sum.push_back(0.0);
    double s_sum = 0;
    for(int i = 0;i<n_fn;i++){
        double x_ = setxy[0][i];
        double y_ = setxy[1][i];
        double s_ = sqrt(pow(x_, 2) + pow(y_, 2));
        s_sum +=s_;
        // std::cout << "s_cSum "<<  s_sum << std::endl;
        s_cum_sum.push_back( s_sum);
    }
    return s_cum_sum; 
}

std::vector<double> CubicSpline2D::calc_s_cumilative_sum( std::vector<std::vector<double>> setxy){
    int n_fn = setxy[0].size() - 1;
    std::vector<double> s_cum_sum;
    s_cum_sum.push_back(0.0);
    double s_sum = 0;
    for(int i = 0;i<n_fn;i++){
        double x_ = setxy[0][i];
        double y_ = setxy[1][i];
        double s_ = sqrt(pow(x_, 2) + pow(y_, 2));
        s_sum +=s_;
        // std::cout << "s_cSum "<<  s_sum << std::endl;
        s_cum_sum.push_back( s_sum);
    }
    return s_cum_sum; 
}

int main(){

std::vector<double> set_x_input{-0.5, 0.0, 0.5, -1.0, -1.5, 3.2};
std::vector<double> set_y_input{3.2, 2.7, 6, 5, 2.5, -1.0};
std::vector<double> set_s_input;
std::vector<std::vector<double>> input_set_xy;
std::vector<std::vector<double>> input_set_sx;
std::vector<std::vector<double>> input_set_sy;
std::vector<std::vector<double>> output_set_xy;
CubicSpline new_setx;
CubicSpline new_sety;

  input_set_xy.push_back(set_x_input);
  input_set_xy.push_back(set_y_input);
 
  set_s_input = new_setx.calc_s_cumilative_sum(input_set_xy);
  input_set_sx.push_back(set_s_input);
  input_set_sx.push_back(set_x_input);

  input_set_sy.push_back(set_s_input);
  input_set_sy.push_back(set_y_input);

  std::vector<std::vector<double>> output_sx_set = new_setx.calc_all(input_set_sx,0.05);
  std::vector<std::vector<double>> output_sy_set = new_sety.calc_all(input_set_sy,0.05);

  output_set_xy.push_back(output_sx_set[1]);
  output_set_xy.push_back(output_sy_set[1]);

  new_setx.write_out_csv("../inputcubic.csv",input_set_xy );
  new_setx.write_out_csv("../cubic.csv",output_set_xy );

}


// } // CubicSplineSolver
