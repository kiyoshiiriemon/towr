/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>
#include <fstream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc

class A1KinematicModel : public KinematicModel {
public:
    A1KinematicModel () : KinematicModel(4)
    {
        const double x_nominal_b = 0.15;
        const double y_nominal_b = 0.12;
        const double z_nominal_b = -0.58;

        nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
        nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
        nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
        nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

        max_dev_from_nominal_ << 0.25, 0.20, 0.10;
    }
};

/**
 * @brief The Dynamics of the quadruped robot A1.
 */
class A1DynamicModel : public SingleRigidBodyDynamics {
public:
    A1DynamicModel() : SingleRigidBodyDynamics(12,
                                               0.946438, 1.94478, 2.01835, 0.000938112, -0.00595386, -0.00146328, // anymal inertia
                                               4) {}
};

class TCStairs : public HeightMap {
public:
    double GetHeight (double x, double y) const override
    {
        double h = 0.0;

        if (x>=first_step_start_)
            h = height_first_step;

        if (x>=first_step_start_+first_step_depth_)
            h = height_second_step;

        if (x>=first_step_start_+first_step_depth_+second_step_depth_)
            h = height_third_step;

        if (x>=first_step_start_+first_step_depth_+second_step_depth_+depth_top)
            h = 0;

        return h;
    }
private:
    double first_step_start_  = 0.5;
    double first_step_depth_  = 0.3;
    double height_first_step  = 0.13;
    double height_second_step = 0.26;
    double second_step_depth_ = 0.3;
    double height_third_step  = 0.39;
    double depth_top = 2.0;
};

int main()
{
  NlpFormulation formulation;

  // terrain
  //formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.terrain_ = std::make_shared<TCStairs>();

  // Kinematic limits and dynamic parameters of the hopper
  formulation.model_ = RobotModel();
  formulation.model_.kinematic_model_ = std::make_shared<A1KinematicModel>();
  formulation.model_.dynamic_model_ = std::make_shared<A1DynamicModel>();

  // set the initial position of the hopper
  formulation.initial_base_.lin.at(kPos).z() = 0.42;
  formulation.initial_ee_W_.push_back(Eigen::Vector3d{0.31,0.19,0.0});
  formulation.initial_ee_W_.push_back(Eigen::Vector3d{0.31,-0.19,0.0});
  formulation.initial_ee_W_.push_back(Eigen::Vector3d{-0.31,0.19,0.0});
  formulation.initial_ee_W_.push_back(Eigen::Vector3d{-0.31,-0.19,0.0});

  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) << 2.0, 0.0, 0.42 + 0.4;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4});
  formulation.params_.ee_phase_durations_.push_back({0.6, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4});
  formulation.params_.ee_phase_durations_.push_back({0.6, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4});
  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4});
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  std::ofstream ofs("motion.txt");
  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
    ofs << solution.base_linear_->GetPoint(t).p().transpose() <<"\t";

      cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    for (auto &ee : solution.ee_motion_)
        ofs << ee->GetPoint(t).p().transpose() << "\t";

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;
    ofs  << endl;

    t += 0.01;
  }
}
