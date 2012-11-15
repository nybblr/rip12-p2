/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors:  Ana Huaman <ahuaman3@gatech.edu>, Tobias Kunz <tobias@gatech.edu>
 * Date: 10/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include "PathPlanner.h"

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner() {
  copyWorld = false;
  world = NULL;
}

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner( robotics::World &_world,
                          bool _copyWorld, double _stepSize ) {

  copyWorld = _copyWorld;

  if( copyWorld ) {
    printf( "Do not use this option yet \n" );
  } else {
    world = &_world;
  }

  stepSize = _stepSize;
}

/**
 * @function ~PathPlanner
 * @brief Destructor
 */
PathPlanner::~PathPlanner() {

  if( copyWorld ) {
    delete world;
  }
}

/**
 * @function planPath
 * @brief Main function
 */
bool PathPlanner::planPath( int _robotId,
                            const Eigen::VectorXi &_links,
                            const Eigen::VectorXd &_start,
                            const Eigen::VectorXd &_goal,
                            bool _bidirectional,
                            bool _connect,
                            bool _greedy,
                            bool _smooth,
                            unsigned int _maxNodes ) {


  //world->mRobots[_robotId]->setQuickDofs( _start ); // Other quick way
  world->getRobot(_robotId)->setDofs( _start, _links );
  if( world->checkCollision() )
    return false;

  world->getRobot(_robotId)->setDofs( _goal, _links );
  if( world->checkCollision() )
    return false;

  bool result;
  if( _bidirectional ) {
    result = planBidirectionalRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
  } else {
    result = planSingleTreeRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
  }

  if( result && _smooth ) {
    smoothPath( _robotId, _links, path );
  }

  return result;
}


/**
 * @function planSingleRRT
 * @brief Finds a plan using a standard RRT
 */
bool PathPlanner::planSingleTreeRrt( int _robotId,
                                     const Eigen::VectorXi &_links,
                                     const Eigen::VectorXd &_start,
                                     const Eigen::VectorXd &_goal,
                                     bool _connect,
                                     bool _greedy,
                                     unsigned int _maxNodes ) {

  RRT rrt( world, _robotId, _links, _start, stepSize );
  RRT::StepResult result = RRT::STEP_PROGRESS;

  double smallestGap = DBL_MAX;

  while ( result != RRT::STEP_REACHED && smallestGap > stepSize ) {

    /** greedy section */
    if( _greedy ) {

      /** greedy and connect */
      if( _connect ) {

  // ================ YOUR CODE HERE ===============
				if(randomInRange(0, 10) < 7)
					rrt.connect();
				else
					rrt.connect(_goal);
  // ===============================================

  /** greedy and NO connect */
      } else {

  // ================== YOUR CODE HERE ===================
				if(randomInRange(0, 10) < 7)
					rrt.tryStep();
				else
					rrt.tryStep(_goal);
  // =====================================================

      }

      /** NO greedy section */
    } else {

      /** NO greedy and Connect */
      if( _connect ) {
  rrt.connect();

  /** No greedy and No connect -- PLAIN RRT */
      } else {
  rrt.tryStep();
      }

    }

    if( _maxNodes > 0 && rrt.getSize() > _maxNodes ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
      return false;
    }

    double gap = rrt.getGap( _goal );
    if( gap < smallestGap ) {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << rrt.configVector.size() << std::endl;
    }
  } // End of while

    /// Save path
  printf(" --> Reached goal! : Gap: %.3f \n", rrt.getGap( _goal ) );
  rrt.tracePath( rrt.activeNode, path, false );

  return true;
}

/**
 * @function planBidirectionalRRT
 * @brief Grows 2 RRT (Start and Goal)
 */
bool PathPlanner::planBidirectionalRrt( int _robotId,
                                        const Eigen::VectorXi &_links,
                                        const Eigen::VectorXd &_start,
                                        const Eigen::VectorXd &_goal,
                                        bool _connect,
                                        bool _greedy, // no effect here
                                        unsigned int _maxNodes ) {

  // ============= YOUR CODE HERE ======================
  // HINT: Remember trees grow towards each other!
	RRT rrts( world, _robotId, _links, _start, stepSize );
	RRT rrtg( world, _robotId, _links, _goal, stepSize );
	RRT::StepResult result = RRT::STEP_PROGRESS;

	double smallestGap = DBL_MAX;
	Eigen::VectorXd sg = _start;
	Eigen::VectorXd gs = _goal;

	while ( result != RRT::STEP_REACHED && (smallestGap > stepSize)) {

    /** greedy section */
    if( _greedy ) {

      /** greedy and connect */
      if( _connect ) {

				if(randomInRange(0, 10) < 7)
				{
					rrts.connect();
					rrtg.connect();
				}
				else
				{
					rrts.connect(gs);
					rrtg.connect(sg);
				}

  /** greedy and NO connect */
      } else {

				if(randomInRange(0, 10) < 7)
				{
					rrts.tryStep();
					rrtg.tryStep();
				}
				else
				{
					rrts.tryStep(gs);
					rrtg.tryStep(sg);
				}

      }

      /** NO greedy section */
    }

		if( _maxNodes > 0 && rrts.getSize() > _maxNodes ) {
			printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
			return false;
		}


		Eigen::VectorXd lastS = rrts.configVector.back();
		Eigen::VectorXd lastG = rrtg.configVector.back();

		int nearestSIdx = rrts.getNearestNeighbor(lastG);
		int nearestGIdx = rrtg.getNearestNeighbor(lastS);

		Eigen::VectorXd nearestS = rrts.configVector[nearestSIdx];
		Eigen::VectorXd nearestG = rrtg.configVector[nearestSIdx];

		double gapS = rrts.getGap( nearestG );
		double gapG = rrtg.getGap( nearestS );

		if( gapG < smallestGap ) {
			smallestGap = gapG;
			sg = lastS;
			gs = lastG;
			std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << rrts.configVector.size()+rrtg.configVector.size() << std::endl;
		}

	} // End of while

		/// Save path
	rrts.tryStep(gs);
	printf(" --> Reached goal! : Gap: %.3f \n", rrts.getGap( gs ) );
	rrts.tracePath( rrts.activeNode, path, false );
	rrtg.tracePath( rrtg.activeNode, path, true );

  return true;
  // ===================================================

}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool PathPlanner::checkPathSegment( int _robotId,
                                    const Eigen::VectorXi &_links,
                                    const Eigen::VectorXd &_config1,
                                    const Eigen::VectorXd &_config2 ) const {

  int n = (int)((_config2 - _config1).norm() / stepSize );

  for( int i = 0; i < n; i++ ) {
    Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
    world->getRobot(_robotId)->setDofs( conf, _links );
    if( world->checkCollision() ) {
      return false;
    }
  }

  return true;
}

/**
 * @function smoothPath
 */
void PathPlanner::smoothPath( int _robotId,
                              const Eigen::VectorXi &_links,
                              std::list<Eigen::VectorXd> &_path ) {

  // =========== YOUR CODE HERE ==================
  // HINT: Use whatever technique you like better, first try to shorten a path and then you can try to make it smoother

  return;
  // ========================================
}


