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

#include "hpp/manipulation/manipulation-planner.hh"

#include <boost/tuple/tuple.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/pointer.hh>
#include <hpp/util/timer.hh>
#include <hpp/util/assertion.hh>

#include <hpp/core/path-validation.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/core/nearest-neighbor.hh>

#include "hpp/manipulation/graph/statistics.hh"
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/node-selector.hh"

namespace hpp {
  namespace manipulation {
    namespace {
      HPP_DEFINE_TIMECOUNTER(oneStep);
      HPP_DEFINE_TIMECOUNTER(extend);
      HPP_DEFINE_TIMECOUNTER(tryConnect);
      HPP_DEFINE_TIMECOUNTER(nearestNeighbor);
      HPP_DEFINE_TIMECOUNTER(delayedEdges);
      HPP_DEFINE_TIMECOUNTER(tryConnectNewNodes);
      HPP_DEFINE_TIMECOUNTER(tryConnectToRoadmap);
      /// extend steps
      HPP_DEFINE_TIMECOUNTER(chooseEdge);
      HPP_DEFINE_TIMECOUNTER(applyConstraints);
      HPP_DEFINE_TIMECOUNTER(buildPath);
      HPP_DEFINE_TIMECOUNTER(projectPath);
      HPP_DEFINE_TIMECOUNTER(validatePath);
    }

    const std::vector<ManipulationPlanner::Reason>
      ManipulationPlanner::reasons_ = boost::assign::list_of
      (SuccessBin::createReason ("[Fail] Projection"))
      (SuccessBin::createReason ("[Fail] SteeringMethod"))
      (SuccessBin::createReason ("[Fail] Path validation returned length 0"))
      (SuccessBin::createReason ("[Fail] Path could not be projected"))
      (SuccessBin::createReason ("[Info] Path could not be fully projected"))
      (SuccessBin::createReason ("[Info] Path could not be fully validated"))
      (SuccessBin::createReason ("[Info] Extended partly"));

    ManipulationPlannerPtr_t ManipulationPlanner::create (const core::Problem& problem,
        const core::RoadmapPtr_t& roadmap)
    {
      ManipulationPlanner* ptr;
      core::RoadmapPtr_t r2 = roadmap;
      RoadmapPtr_t r;
      try {
        const Problem& p = dynamic_cast < const Problem& > (problem);
        RoadmapPtr_t r = HPP_DYNAMIC_PTR_CAST (Roadmap, r2);
        ptr = new ManipulationPlanner (p, r);
      } catch (std::exception&) {
        if (!r)
          throw std::invalid_argument ("The roadmap must be of type hpp::manipulation::Roadmap.");
        else
        throw std::invalid_argument ("The problem must be of type hpp::manipulation::Problem.");
      }
      ManipulationPlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    bool belongs (const ConfigurationPtr_t& q, const core::Nodes_t& nodes)
    {
      for (core::Nodes_t::const_iterator itNode = nodes.begin ();
          itNode != nodes.end (); ++itNode) {
        if (*((*itNode)->configuration ()) == *q) return true;
      }
      return false;
    }

    ManipulationPlanner::ErrorFreqs_t ManipulationPlanner::getEdgeStat
      (const graph::EdgePtr_t& edge) const
    {
      const std::size_t& id = edge->id ();
      ErrorFreqs_t ret;
      if (indexPerEdgeStatistics_.size() <= id ||
         indexPerEdgeStatistics_[id] < 0) {
        for (int i = 0; i < 6; ++i) ret.push_back (0);
      } else {
        const SuccessStatistics& ss =
          perEdgeStatistics_[indexPerEdgeStatistics_[id]];
        ret.push_back (ss.nbSuccess ());
        for (int i = 0; i < 6; ++i)
          ret.push_back (ss.nbFailure (reasons_[i]));
      }
      return ret;
    }

    StringList_t ManipulationPlanner::errorList ()
    {
      StringList_t ret;
      ret.push_back ("Success");
      for (int i = 0; i < 6; ++i) ret.push_back (reasons_[i].what);
      return ret;
    }

    void ManipulationPlanner::oneStep ()
    {
      HPP_START_TIMECOUNTER(oneStep);

      DevicePtr_t robot = HPP_DYNAMIC_PTR_CAST(Device, problem ().robot ());
      HPP_ASSERT(robot);
      const graph::Nodes_t& graphNodes = problem_.constraintGraph ()
        ->nodeSelector ()->getNodes ();
      graph::Nodes_t::const_iterator itNode;
      core::Nodes_t newNodes;
      core::PathPtr_t path;

      typedef boost::tuple <core::NodePtr_t, ConfigurationPtr_t, core::PathPtr_t>
	DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;

      // Pick a random node
      ConfigurationPtr_t q_rand = shooter_->shoot();

      // Extend each connected component
      for (core::ConnectedComponents_t::const_iterator itcc =
          roadmap ()->connectedComponents ().begin ();
          itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
        // Find the nearest neighbor.
        core::value_type distance;
        for (itNode = graphNodes.begin (); itNode != graphNodes.end (); ++itNode) {
          HPP_START_TIMECOUNTER(nearestNeighbor);
          RoadmapNodePtr_t near = roadmap_->nearestNode (q_rand, HPP_STATIC_PTR_CAST(ConnectedComponent,*itcc), *itNode, distance);
          HPP_STOP_TIMECOUNTER(nearestNeighbor);
          HPP_DISPLAY_LAST_TIMECOUNTER(nearestNeighbor);
          if (!near) continue;

          HPP_START_TIMECOUNTER(extend);
          bool pathIsValid = extend (near, q_rand, path);
          HPP_STOP_TIMECOUNTER(extend);
          HPP_DISPLAY_LAST_TIMECOUNTER(extend);
          // Insert new path to q_near in roadmap
          if (pathIsValid) {
            value_type t_final = path->timeRange ().second;
            if (t_final != path->timeRange ().first) {
	      bool success;
              ConfigurationPtr_t q_new (new Configuration_t
					((*path) (t_final, success)));
              if (!belongs (q_new, newNodes)) {
                newNodes.push_back (roadmap ()->addNodeAndEdges
                    (near, q_new, path));
              } else {
                delayedEdges.push_back (DelayedEdge_t (near, q_new, path));
              }
            }
            
          }


        }
      }

      HPP_START_TIMECOUNTER(delayedEdges);
      // Insert delayed edges
      for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
	   itEdge != delayedEdges.end (); ++itEdge) {
	const core::NodePtr_t& near = itEdge-> get <0> ();
	const ConfigurationPtr_t& q_new = itEdge-> get <1> ();
	const core::PathPtr_t& validPath = itEdge-> get <2> ();
        core::NodePtr_t newNode = roadmap ()->addNode (q_new);
	roadmap ()->addEdge (near, newNode, validPath);
        core::interval_t timeRange = validPath->timeRange ();
	roadmap ()->addEdge (newNode, near, validPath->extract
			     (core::interval_t (timeRange.second ,
					  timeRange.first)));
      }
      HPP_STOP_TIMECOUNTER(delayedEdges);

      // Try to connect the new nodes together
      HPP_START_TIMECOUNTER(tryConnectNewNodes);
      const std::size_t nbConn = tryConnectNewNodes (newNodes);
      HPP_STOP_TIMECOUNTER(tryConnectNewNodes);
      HPP_DISPLAY_LAST_TIMECOUNTER(tryConnectNewNodes);
      if (nbConn == 0) {
        HPP_START_TIMECOUNTER(tryConnectToRoadmap);
        tryConnectToRoadmap (newNodes);
        HPP_STOP_TIMECOUNTER(tryConnectToRoadmap);
        HPP_DISPLAY_LAST_TIMECOUNTER(tryConnectToRoadmap);
      }
      HPP_STOP_TIMECOUNTER(oneStep);
      HPP_DISPLAY_LAST_TIMECOUNTER(oneStep);
      HPP_DISPLAY_TIMECOUNTER(oneStep);
      HPP_DISPLAY_TIMECOUNTER(extend);
      HPP_DISPLAY_TIMECOUNTER(tryConnect);
      HPP_DISPLAY_TIMECOUNTER(tryConnectNewNodes);
      HPP_DISPLAY_TIMECOUNTER(tryConnectToRoadmap);
      HPP_DISPLAY_TIMECOUNTER(nearestNeighbor);
      HPP_DISPLAY_TIMECOUNTER(delayedEdges);
      HPP_DISPLAY_TIMECOUNTER(chooseEdge);
      HPP_DISPLAY_TIMECOUNTER(applyConstraints);
      HPP_DISPLAY_TIMECOUNTER(buildPath);
      HPP_DISPLAY_TIMECOUNTER(projectPath);
      HPP_DISPLAY_TIMECOUNTER(validatePath);
    }

    bool ManipulationPlanner::extend(
        RoadmapNodePtr_t n_near,
        const ConfigurationPtr_t& q_rand,
        core::PathPtr_t& validPath)
    {
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      PathProjectorPtr_t pathProjector = problem_.pathProjector ();
      // Select next node in the constraint graph.
      const ConfigurationPtr_t q_near = n_near->configuration ();
      HPP_START_TIMECOUNTER (chooseEdge);
      graph::EdgePtr_t edge = graph->chooseEdge (n_near);
      HPP_STOP_TIMECOUNTER (chooseEdge);
      if (!edge) {
        return false;
      }
      qProj_ = *q_rand;
      HPP_START_TIMECOUNTER (applyConstraints);
      SuccessStatistics& es = edgeStat (edge);
      if (!edge->applyConstraints (n_near, qProj_)) {
        HPP_STOP_TIMECOUNTER (applyConstraints);
        es.addFailure (reasons_[PROJECTION]);
        return false;
      }
      HPP_STOP_TIMECOUNTER (applyConstraints);
      core::PathPtr_t path;
      HPP_START_TIMECOUNTER (buildPath);
      if (!edge->build (path, *q_near, qProj_)) {
        HPP_STOP_TIMECOUNTER (buildPath);
        es.addFailure (reasons_[STEERING_METHOD]);
        return false;
      }
      HPP_STOP_TIMECOUNTER (buildPath);
      core::PathPtr_t projPath;
      bool projShorter = false;
      if (pathProjector) {
        HPP_START_TIMECOUNTER (projectPath);
        projShorter = !pathProjector->apply (path, projPath);
        if (projShorter) {
          if (!projPath || projPath->length () == 0) {
            HPP_STOP_TIMECOUNTER (projectPath);
            es.addFailure (reasons_[PATH_PROJECTION_ZERO]);
            return false;
          }
          es.addFailure (reasons_[PATH_PROJECTION_SHORTER]);
        }
        HPP_STOP_TIMECOUNTER (projectPath);
      } else projPath = path;
      GraphPathValidationPtr_t pathValidation (problem_.pathValidation ());
      PathValidationReportPtr_t report;
      core::PathPtr_t fullValidPath;
      HPP_START_TIMECOUNTER (validatePath);
      bool fullyValid = false;
      try {
        fullyValid = pathValidation->validate
          (projPath, false, fullValidPath, report);
      } catch (const core::projection_error& e) {
        hppDout (error, e.what ());
        es.addFailure (reasons_[PATH_VALIDATION_ZERO]);
        return false;
      }
      HPP_STOP_TIMECOUNTER (validatePath);
      if (fullValidPath->length () == 0) {
        es.addFailure (reasons_[PATH_VALIDATION_ZERO]);
        validPath = fullValidPath;
        return false;
      } else {
        if (!fullyValid) es.addFailure (reasons_[PATH_VALIDATION_SHORTER]);
        if (extendStep_ == 1 || fullyValid) {
          validPath = fullValidPath;
        } else {
          const value_type& length = fullValidPath->length();
          const value_type& t_init = fullValidPath->timeRange ().first;
          try {
            validPath = fullValidPath->extract
              (core::interval_t(t_init, t_init + length * extendStep_));
          } catch (const core::projection_error& e) {
            hppDout (error, e.what());
            es.addFailure (reasons_[PATH_PROJECTION_SHORTER]);
            return false;
          }
        }
        hppDout (info, "Extension:" << std::endl
            << es);
      }
      if (!projShorter && fullValidPath) es.addSuccess ();
      else es.addFailure (reasons_[PARTLY_EXTENDED]);
      return true;
    }

    ManipulationPlanner::SuccessStatistics& ManipulationPlanner::edgeStat
      (const graph::EdgePtr_t& edge)
    {
      const std::size_t& id = edge->id ();
      if (indexPerEdgeStatistics_.size () <= id) {
        indexPerEdgeStatistics_.resize (id + 1, -1);
      }
      if (indexPerEdgeStatistics_[id] < 0) {
        indexPerEdgeStatistics_[id] = perEdgeStatistics_.size();
        perEdgeStatistics_.push_back (SuccessStatistics (edge->name (), 2));
      }
      return perEdgeStatistics_[indexPerEdgeStatistics_[id]];
    }

    inline std::size_t ManipulationPlanner::tryConnectToRoadmap (const core::Nodes_t nodes)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      core::PathPtr_t path, projPath, validPath;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      bool connectSucceed = false;
      std::size_t nbConnection = 0;
      const std::size_t K = 7;
      value_type distance;
      for (core::Nodes_t::const_iterator itn1 = nodes.begin ();
          itn1 != nodes.end (); ++itn1) {
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
        connectSucceed = false;
        for (core::ConnectedComponents_t::const_iterator itcc =
            roadmap ()->connectedComponents ().begin ();
            itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
          if (*itcc == (*itn1)->connectedComponent ())
            continue;
          core::Nodes_t knearest = roadmap()->nearestNeighbor ()
            ->KnearestSearch (q1, *itcc, K, distance);
          for (core::Nodes_t::const_iterator itn2 = knearest.begin ();
              itn2 != knearest.end (); ++itn2) {
            bool _1to2 = (*itn1)->isOutNeighbor (*itn2);
            bool _2to1 = (*itn1)->isInNeighbor (*itn2);
            if (_1to2 && _2to1) {
              hppDout (info, "the two nodes are already connected");
              continue;
            }
            ConfigurationPtr_t q2 ((*itn2)->configuration ());
            assert (*q1 != *q2);
            path = (*sm) (*q1, *q2);
            if (!path) continue;
            if (pathProjector) {
              if (!pathProjector->apply (path, projPath)) continue;
            } else projPath = path;
	    PathValidationReportPtr_t report;
            if (pathValidation->validate (projPath, false, validPath, report)) {
              nbConnection++;
              if (!_1to2) roadmap ()->addEdge (*itn1, *itn2, projPath);
              if (!_2to1) {
                core::interval_t timeRange = projPath->timeRange ();
                roadmap ()->addEdge (*itn2, *itn1, projPath->extract
                    (core::interval_t (timeRange.second,
                                       timeRange.first)));
              }
              connectSucceed = true;
              break;
            }
          }
          if (connectSucceed) break;
        }
      }
      return nbConnection;
    }

    inline std::size_t ManipulationPlanner::tryConnectNewNodes (const core::Nodes_t nodes)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      PathProjectorPtr_t pathProjector (problem().pathProjector ());
      core::PathPtr_t path, projPath, validPath;
      graph::GraphPtr_t graph = problem_.constraintGraph ();
      std::size_t nbConnection = 0;
      for (core::Nodes_t::const_iterator itn1 = nodes.begin ();
          itn1 != nodes.end (); ++itn1) {
        ConfigurationPtr_t q1 ((*itn1)->configuration ());
        for (core::Nodes_t::const_iterator itn2 = boost::next (itn1);
            itn2 != nodes.end (); ++itn2) {
          if ((*itn1)->connectedComponent () == (*itn2)->connectedComponent ())
            continue;
          bool _1to2 = (*itn1)->isOutNeighbor (*itn2);
          bool _2to1 = (*itn1)->isInNeighbor (*itn2);
          if (_1to2 && _2to1) {
            hppDout (info, "the two nodes are already connected");
            continue;
          }
          ConfigurationPtr_t q2 ((*itn2)->configuration ());
          assert (*q1 != *q2);
          path = (*sm) (*q1, *q2);
          if (!path) continue;
          if (pathProjector) {
            if (!pathProjector->apply (path, projPath)) continue;
          } else projPath = path;
          PathValidationReportPtr_t report;
          if (pathValidation->validate (projPath, false, validPath, report)) {
            nbConnection++;
            if (!_1to2) roadmap ()->addEdge (*itn1, *itn2, projPath);
            if (!_2to1) {
              core::interval_t timeRange = projPath->timeRange ();
              roadmap ()->addEdge (*itn2, *itn1, projPath->extract
                  (core::interval_t (timeRange.second,
                                     timeRange.first)));
            }
          }
        }
      }
      return nbConnection;
    }

    ManipulationPlanner::ManipulationPlanner (const Problem& problem,
        const RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap),
      shooter_ (problem.configurationShooter()),
      problem_ (problem), roadmap_ (roadmap),
      extendStep_ (1),
      qProj_ (problem.robot ()->configSize ())
    {}

    void ManipulationPlanner::init (const ManipulationPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }
  } // namespace manipulation
} // namespace hpp
