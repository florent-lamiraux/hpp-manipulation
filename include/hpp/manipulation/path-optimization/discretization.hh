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

#ifndef HPP_MANIPULATION_PATH_OPTIMIZATION_DISCRETIZATION_HH
#define HPP_MANIPULATION_PATH_OPTIMIZATION_DISCRETIZATION_HH

#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/core/path-optimizer.hh>

namespace hpp{
namespace manipulation{
namespace pathOptimization{
  /// \addtogroup path_optimization
  /// \{

  /// Discretize a path and store configs in a core::InterpolatedPath
  ///
  /// When a path is subject to nonlinear constraints, online evaluation
  /// of configurations along the path requires resolution of the nonlinear
  /// constraints. In some cases, this prevents to perform online
  /// discretization at high rates.
  ///
  /// The constraints of each path in the input path are passed to the
  /// interpolated paths of the output. This is particularly useful to keep
  /// track of the transition that created the path.
  class HPP_MANIPULATION_DLLAPI Discretization : public core::PathOptimizer
  {
  public:
    static DiscretizationPtr_t create(const core::ProblemConstPtr_t& problem);
    /// Optimize path
    virtual core::PathVectorPtr_t optimize(const core::PathVectorPtr_t& path);
  protected:
    Discretization(const ProblemConstPtr_t& problem);
  }; // class Discretization

  /// \}
} // namespace pathOptimization
} // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PATH_OPTIMIZATION_DISCRETIZATION_HH
