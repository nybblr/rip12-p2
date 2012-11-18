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

  int randomCount = 0;
  int goalCount = 0;
  double smallestGap = DBL_MAX;

  while ( result != RRT::STEP_REACHED && smallestGap > stepSize ) {

    /** greedy section */
    if( _greedy ) {

      /** greedy and connect */
      if( _connect ) {

  // ================ YOUR CODE HERE ===============
				if(randomInRange(0, 100) < 65)
				{
					rrt.connect();
					randomCount++;
				}
				else
				{
					goalCount++;
					rrt.connect(_goal);
				}
  // ===============================================

  /** greedy and NO connect */
      } else {

  // ================== YOUR CODE HERE ===================
				if(randomInRange(0, 100) < 65)
				{
					rrt.tryStep();
					randomCount++;
				}
				else
				{
					rrt.tryStep(_goal);
					goalCount++;
				}
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
	  printf("--(!) Random Count: %d \n", randomCount );
	  printf("--(!) Goal Count: %d \n", goalCount );
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
  printf("--(!) Random Count: %d \n", randomCount );
  printf("--(!) Goal Count: %d \n", goalCount );
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
	int randomCount = 0;
    int goalCount = 0;
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
					randomCount++;
				}
				else
				{
					rrts.connect(rrtg.configVector[rrtg.activeNode]);
					rrtg.connect(rrts.configVector[rrts.activeNode]);
					goalCount++;
				}

  /** greedy and NO connect */
      } else {

				if(randomInRange(0, 10) < 7)
				{
					rrts.tryStep();
					rrtg.tryStep();
					randomCount++;
				}
				else
				{
					rrts.tryStep(rrtg.configVector[rrtg.activeNode]);
					rrtg.tryStep(rrts.configVector[rrts.activeNode]);
					goalCount++;
				}

      }

      /** NO greedy section */
    }

		if( _maxNodes > 0 && rrts.getSize() > _maxNodes ) {
			printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
			printf("--(!) Random Count: %d \n", randomCount );
            printf("--(!) Goal Count: %d \n", goalCount );
			return false;
		}


		Eigen::VectorXd lastS = rrts.configVector.back();
		Eigen::VectorXd lastG = rrtg.configVector.back();

		int nearestSIdx = rrts.getNearestNeighbor(lastG);
		int nearestGIdx = rrtg.getNearestNeighbor(lastS);

		Eigen::VectorXd nearestS = rrts.configVector[nearestSIdx];
		Eigen::VectorXd nearestG = rrtg.configVector[nearestGIdx];

		double gapS = rrts.getGap( nearestG );
		double gapG = rrtg.getGap( nearestS );

		if( gapG < smallestGap ) {
			smallestGap = gapG;
			std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << (rrts.configVector.size()+rrtg.configVector.size()) << std::endl;
		}

		if( gapS < smallestGap ) {
			smallestGap = gapS;
			std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << (rrts.configVector.size()+rrtg.configVector.size()) << std::endl;
		}

	} // End of while

		/// Save path
	printf(" --> Reached goal! : Gap: %.3f \n", smallestGap );
	printf("--(!) Random Count: %d \n", randomCount );
    printf("--(!) Goal Count: %d \n", goalCount );
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

	bool clear = checkPathSegment(_robotId, _links, _path.front(), _path.back());
	std::cout << "Start: " << _path.front() << " End: " << _path.back() << std::endl;
	printf("Path clear? %s\n", clear ? "true" : "false");
	const int MAX_TRIES = 100;
	// Strategy: randomly pick a pair of nodes (within some distance threshold) and try to connect them
	// If there is no collision, then replace all in between nodes with a straight line.

	for( int i = 0; i < MAX_TRIES; i++ ) {
		// First: get random nodes in some distance range
		int IDa = 0;
		int IDb = 0;
		while( true ) {
			IDa = randomInRange(0, _path.size() - 1);
			IDb = randomInRange(0, _path.size() - 1);

			int IDd = abs(IDa - IDb);
			if( IDd > 3 && IDd < 250 )
				break;
		}

		// Ensure IDa < IDb
		if( IDa > IDb ) {
			// Swap
			int IDt = IDa;
			IDa = IDb;
			IDb = IDt;
		}

		std::list<Eigen::VectorXd>::iterator nodeAit = std::next(_path.begin(), IDa);
		Eigen::VectorXd nodeA = *nodeAit;
		std::list<Eigen::VectorXd>::iterator nodeBit = std::next(_path.begin(), IDb);
		Eigen::VectorXd nodeB = *nodeBit;

		printf("Random pair %d of %d: %d %d\n", i, MAX_TRIES, IDa, IDb);

		printf("Current size: %d\n", _path.size());
		clear = checkPathSegment(_robotId, _links, nodeA, nodeB);
		printf("Path clear? %s\n", clear ? "true" : "false");

		if( clear ) {
			// Collision free!
			// First, remove in between nodes
			std::advance(nodeAit, 1);
			_path.erase(nodeAit, nodeBit);

			Eigen::VectorXd start = nodeA;
			Eigen::VectorXd end = nodeB;

			// Now insert straight line nodes
			while( true ) {
				// Compute direction and magnitude
				Eigen::VectorXd diff = end - start;
				double dist = diff.norm();

				if( dist < stepSize )
					break;

				// Scale this vector to stepSize and add to end of start
				Eigen::VectorXd qnew = start + diff*(stepSize/dist);
				// Insert before endpoint
				_path.insert(nodeBit, qnew);

				// Increment "counter"
				start = qnew;
			}
			printf("New size: %d\n", _path.size());
		} else {
			printf("Collision.\n");
		}
	}

  return;
  // ========================================
}


