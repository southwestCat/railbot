#include "MPC.h"
#include <cassert>
#include <iostream>

using namespace Eigen;
using namespace std;

void Point::integrateConstantAcceleration(Vector3f pdd, float dt)
{
    this->p_ = this->p_ + (this->pd_ + 0.5 * pdd * dt) * dt;
    this->pd_ = this->pd_ + pdd * dt;
    this->pdd_ = pdd;
}

void Point::integrateConstantJerk(Vector3f pddd, float dt)
{
    this->p_ = this->p_ + dt * (this->pd_ + 0.5 * dt * (this->pdd_ + dt * pddd / 3.0));
    this->pd_ = this->pd_ + dt * (this->pdd_ + dt * pddd / 2.0);
    this->pdd_ = this->pdd_ + dt * pddd;
}

MPC::MPC(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D,
         std::vector<Vector2f> e, Eigen::MatrixXf x_init, Eigen::MatrixXf x_goal, unsigned nb_steps, float wxt, float wxc, float wu)
{
    this->A = A;
    this->B = B;
    this->C = C;
    this->D = D;
    this->e = e;
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->nb_steps = nb_steps;
    this->wxt = wxt;
    this->wxc = wxc;
    this->wu = wu;

    build();
}

void MPC::build()
{
    assert(e.size() == nb_steps);

    //! || P*x+q ||^2
    //! min {} 1/2 x'*Q*x + c'*x } with A_ineq * x \leq b_ineq;
    //! Q = P'*P, c = P'*q
    const int NB_VAR = nb_steps;
    const int COST_DIM = 3 * NB_VAR + 1;
    MatrixXf P;
    VectorXf q;
    P.setZero(COST_DIM, NB_VAR);
    q.setZero(COST_DIM);
    const int NB_CONS = 2 * NB_VAR;
    qpSolver.problem(NB_VAR, 0, NB_CONS);
    this->A_ineq.resize(NB_CONS, NB_VAR);
    this->b_ineq.resize(NB_CONS);
    this->A_eq.resize(0, 0);
    this->b_eq.resize(0);
    MatrixXf psi;
    Matrix3f phi = A;
    psi.setZero(3, NB_VAR);
    for (int k = 0; k < NB_VAR; k++)
    {
        psi = A * psi;
        psi.block(0, k, 3, 1) = B;

        auto Pk = P.block(k * 3, 0, 3, NB_VAR);
        auto qk = q.segment<3>(k * 3);
        Pk = k == (NB_VAR - 1) ? psi * wxt : psi * wxc;
        qk = k == (NB_VAR - 1) ? (phi * x_init - x_goal) * wxt : (phi * x_init - x_goal) * wxc;

        auto A_ineq_k = A_ineq.block(k * 2, 0, 2, NB_VAR);
        auto b_ineq_k = b_ineq.segment<2>(k * 2);
        A_ineq_k = C.cast<double>() * psi.cast<double>();
        b_ineq_k = e.at(k).cast<double>() - C.cast<double>() * phi.cast<double>() * x_init.cast<double>();

        phi *= A;
    }
    for (int i = 0; i < NB_VAR; i++)
    {
        P(COST_DIM - 1, i) = 1.f * wu;
    }
    // cout << P << endl << endl;
    // cout << x_init.transpose() << " " << x_goal.transpose() << endl;

    this->Q = P.transpose().cast<double>() * P.cast<double>();
    this->c = P.transpose().cast<double>() * q.cast<double>();
}

bool MPC::solve()
{
    bool solution = qpSolver.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq);
    if (!solution)
    {
        cout << "Cannot find QP solution." << endl;
    }
    return solution;
}
