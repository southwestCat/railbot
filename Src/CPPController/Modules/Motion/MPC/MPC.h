#pragma once

#include <array>
#include <vector>
#include <eigen-quadprog/QuadProg.h>
#include "Tools/Math/Eigen.h"

class Point
{
public:
    Point() = default;
    Point(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f accel)
    {
        this->p_ = pos;
        this->pd_ = vel;
        this->pdd_ = accel;
    }

    Eigen::Vector3f p() { return this->p_; }
    float x() { return this->p_[0]; }
    float y() { return this->p_[1]; }
    float z() { return this->p_[2]; }

    Eigen::Vector3f pd() { return this->pd_; }
    float xd() { return this->pd_[0]; }
    float yd() { return this->pd_[1]; }
    float zd() { return this->pd_[2]; }
    void setVelocity(Eigen::Vector3f pd) { this->pd_ = pd; }

    Eigen::Vector3f pdd() { return this->pdd_; }
    float xdd() { return this->pdd_[0]; }
    float ydd() { return this->pdd_[1]; }
    float zdd() { return this->pdd_[2]; }
    void setAcceleration(Eigen::Vector3f acc) { this->pdd_ = acc; }

    void integrateConstantAcceleration(Eigen::Vector3f pdd, float dt);
    void integrateConstantJerk(Eigen::Vector3f pddd, float dt);

private:
    Eigen::Vector3f p_, pd_, pdd_;
};

class MPC
{
public:
    MPC() {}
    MPC(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D,
        std::vector<Eigen::Vector2f> e, Eigen::MatrixXf x_init, Eigen::MatrixXf x_goal, unsigned nb_steps, float wxt, float wxc, float wu);

    bool solve();
    const Eigen::VectorXd result() const { return qpSolver.result(); }

private:
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;
    Eigen::MatrixXf x_init;
    Eigen::MatrixXf x_goal;

    Eigen::MatrixXd Q;
    Eigen::VectorXd c;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;
    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;

    std::vector<Eigen::Vector2f> e;

    Eigen::QuadProgDense qpSolver;

    unsigned nb_steps;

    float wxt;
    float wxc;
    float wu;

    void build();
};
