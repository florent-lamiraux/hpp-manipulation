//
// Copyright (c) 2022 CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <hpp/core/config-projector.hh>
#include <hpp/core/interpolated-path.hh>
#include <hpp/core/path-vector.hh>

#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/path-optimization/discretization.hh>
#include <hpp/manipulation/problem.hh>

namespace hpp{
namespace manipulation{
namespace pathOptimization{
  using core::PathVectorPtr_t;
  using core::PathVector;
  using core::Parameter;
  using core::ParameterDescription;

namespace internal{
  // Constraint set that derives from manipulation::ConstraintSet, but that
  // does not apple the constraints
  class ConstraintSet : public manipulation::ConstraintSet
  {
  public:
    typedef shared_ptr<ConstraintSet> Ptr_t;
    static Ptr_t create(const manipulation::ConstraintSetPtr_t& cs)
    {
      return Ptr_t(new ConstraintSet(cs));
    }
    virtual bool impl_compute(ConfigurationOut_t /*configuration*/)
    {
      std::cout << "impl_compute does nothing." << std::endl;
      // does nothing
      return true;
    }
  protected:
    ConstraintSet(const manipulation::ConstraintSetPtr_t& cs) :
      manipulation::ConstraintSet(HPP_DYNAMIC_PTR_CAST
	  (manipulation::Device, cs->configProjector()->robot()),
	  cs->name() + std::string(" (fake)"))
    {
      // Set edge as the same as input constraint set
      this->edge(cs->edge());
    }
  }; // class ConstraintSet

} // namespace internal

  DiscretizationPtr_t Discretization::create
  (const core::ProblemConstPtr_t& problem)
  {
    ProblemConstPtr_t p(HPP_DYNAMIC_PTR_CAST(const Problem, problem));
    if (!p) throw std::invalid_argument
              ("hpp::manipulation::pathOptimizer::Discretization constructor "
               "expects a manipulation::Problem as input.");
    return DiscretizationPtr_t(new Discretization(p));
  }

  Discretization::Discretization(const ProblemConstPtr_t& problem):
    core::PathOptimizer(problem)
  {
  }

  PathVectorPtr_t Discretization::optimize(const PathVectorPtr_t& path)
  {
    // Find discretization step in parameter map
    value_type step =
    problem()->getParameter("PathOptimization/Discretization/step").
      floatValue();
    PathVectorPtr_t output =
      PathVector::create(path->outputSize(), path->outputDerivativeSize());

    for (std::size_t i=0; i < path->numberPaths(); ++i){
      core::PathPtr_t p(path->pathAtRank(i));
      ConstraintSetPtr_t mcs(HPP_DYNAMIC_PTR_CAST(manipulation::ConstraintSet,
						  p->constraints()));
      if (!mcs) {
	throw std::logic_error("hpp::manipulation::Discretization::optimize: "
			       "ConstraintSet is not of type manipulation::"
			       "ConstraintSet");
      }
      core::InterpolatedPathPtr_t discretized = core::InterpolatedPath::create
        (problem()->robot(), p->initial(), p->end(), p->length(),
         internal::ConstraintSet::create(mcs));
      value_type t = p->timeRange().first + step;
      while (t < p->timeRange().second - 1e-8){
        bool success;
        discretized->insert(t, p->eval(t, success));
        if (!success){
          std::ostringstream os;
          os << "Failed to evaluate path at rank " << i << " at parameter "
             << t;
          throw std::runtime_error(os.str().c_str());
        }
        t += step;
      }
      assert(p->initial() == discretized->initial());
      assert(p->end() == discretized->end());

      output->appendPath(discretized);
    }
    return output;
  }

  HPP_START_PARAMETER_DECLARATION(Discretization)
  core::Problem::declareParameter(ParameterDescription(
    Parameter::FLOAT, "PathOptimization/Discretization/step",
    "Time step for discretization.", Parameter(1e-3)));
  HPP_END_PARAMETER_DECLARATION(Discretization)
} // namespace pathOptimization
} // namespace manipulation
} // namespace hpp
