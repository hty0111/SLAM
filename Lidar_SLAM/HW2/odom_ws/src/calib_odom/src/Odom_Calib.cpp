#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        A(now_len * 3, 0) = Odom(0);
        A(now_len * 3, 1) = Odom(1);
        A(now_len * 3, 2) = Odom(2);
        A(now_len * 3 + 1, 3) = Odom(0);
        A(now_len * 3 + 1, 4) = Odom(1);
        A(now_len * 3 + 1, 5) = Odom(2);
        A(now_len * 3 + 2, 6) = Odom(0);
        A(now_len * 3 + 2, 7) = Odom(1);
        A(now_len * 3 + 2, 8) = Odom(2);
        b(now_len * 3) = scan(0);
        b(now_len * 3 + 1) = scan(1);
        b(now_len * 3 + 1) = scan(2);
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    int len = now_len - 1;
    A.conservativeResize(len * 3, 9);   // 仅保留有数据的部分
    b.conservativeResize(len * 3);
    Eigen::MatrixXd ATA = A.transpose() * A;
    Eigen::MatrixXd ATb = A.transpose() * b;

    Eigen::VectorXd x, x1, x2, x3, x4, x5;

    x1 = ATA.inverse() * ATb;   // 最小二乘公式
    x2 = ATA.lu().solve(ATb);   // LU分解，A要可逆
    x3 = ATA.llt().solve(ATb);  // Cholesky分解，A要正定
    x4 = A.householderQr().solve(b);    // QR分解
    x5 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);   // 奇异值分解

    x = x1;
    correct_matrix << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8);
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
