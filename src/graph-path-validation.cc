// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/manipulation/graph-path-validation.hh"

namespace hpp {
  namespace manipulation {
    GraphPathValidationPtr_t GraphPathValidation::create (
        const PathValidationPtr_t& pathValidation, const GraphPtr_t& graph)
    {
      GraphPathValidation* p = new GraphPathValidation (pathValidation, graph);
      return GraphPathValidationPtr_t (p);
    }

    GraphPathValidation::GraphPathValidation (
        const PathValidationPtr_t& pathValidation, const GraphPtr_t& graph) :
      pathValidation_ (pathValidation), constraintGraph_ (graph)
    {}

    bool GraphPathValidation::validate (
          const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      assert (path);
      return impl_validate (path, reverse, validPart);
    }

    bool GraphPathValidation::impl_validate (
        const PathVectorPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      PathPtr_t validSubPart;
      if (reverse) {
        // TODO: This has never been tested.
        for (int i = path->numberPaths () - 1; i >= 0; i--) {
          // We should stop at the first non valid subpath.
          if (!impl_validate (path->pathAtRank (i), true, validSubPart)) {
            PathVectorPtr_t p = PathVector::create (path->outputSize());
            for (int v = path->numberPaths () - 1; v > i; v--)
              p->appendPath (path->pathAtRank(i)->copy());
            // TODO: Make sure this subpart is generated by the steering method.
            p->appendPath (validSubPart);
            validPart = p;
            return false;
          }
        }
      } else {
        for (size_t i = 0; i != path->numberPaths (); i++) {
          // We should stop at the first non valid subpath.
          if (!impl_validate (path->pathAtRank (i), false, validSubPart)) {
            PathVectorPtr_t p = PathVector::create (path->outputSize());
            for (size_t v = 0; v < i; v++)
              p->appendPath (path->pathAtRank(i)->copy());
            // TODO: Make sure this subpart is generated by the steering method.
            p->appendPath (validSubPart);
            validPart = p;
            return false;
          }
        }
      }
      // Here, every subpath is valid.
      validPart = path;
      return true;
    }

    bool GraphPathValidation::impl_validate (
        const PathPtr_t& path, bool reverse, PathPtr_t& validPart)
    {
      PathVectorPtr_t pathVector = HPP_DYNAMIC_PTR_CAST(PathVector, path);
      if (pathVector)
        return impl_validate (pathVector, reverse, validPart);

      PathPtr_t pathNoCollision;
      if (pathValidation_->validate (path, reverse, pathNoCollision)) {
        validPart = path;
        return true;
      }
      const Path& newPath (*pathNoCollision);
      value_type newTmin = pathNoCollision->timeRange ().first,
                 newTmax = pathNoCollision->timeRange ().second,
                 oldTmin = path->timeRange ().first,
                 oldTmax = path->timeRange ().second;
      graph::Nodes_t origNodes, destNodes;
      try {
        origNodes = constraintGraph_->getNode (newPath (newTmin));
        destNodes = constraintGraph_->getNode (newPath (newTmax));

        if (origNodes == constraintGraph_->getNode ((*path) (oldTmin))
            && destNodes == constraintGraph_->getNode ((*path) (oldTmax))) {
          validPart = pathNoCollision;
          return false;
        }
      } catch (std::logic_error& e) {
        /// A configuration has no node, which may be because the path could not be projected.
        /// Path should be considered invalid.
        validPart = path->extract (std::make_pair (oldTmin,oldTmin));
        return false;
      }

      // Here, the full path is not valid. Moreover, it does not correspond to the same
      // edge of the constraint graph. Two option are possible:
      // - Use the steering method to create a new path and validate it.
      // - Return a null path.
      assert (!reverse && "This has never been tested with reverse path");
      std::vector <graph::Edges_t> possibleEdges (constraintGraph_->getEdge (origNodes, destNodes));
      // We check for all of them if both nodes are on the same leaf.
      ConstraintSetPtr_t constraints;
      while (!possibleEdges.empty ()) {
        constraints = constraintGraph_->pathConstraint (possibleEdges.back());
        constraints->offsetFromConfig(newPath (newTmin));
        assert (constraints->isSatisfied (newPath (newTmin)));
        if (constraints->isSatisfied (newPath (newTmax))) {
          pathNoCollision->constraints (constraints);
          impl_validate (pathNoCollision, reverse, validPart);
          return false;
        }
        possibleEdges.pop_back();
      }
      validPart = path->extract (std::make_pair (oldTmin,oldTmin));
      return false;
    }
  } // namespace manipulation
} // namespace hpp
