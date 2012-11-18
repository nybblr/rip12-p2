/**
 * @file JT_Follower.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date March 07th, 2012
 */

#include <boost/algorithm/string.hpp>
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "JTFollower.h"

#include <Eigen/LU>
#include "spnav.h"
#include <fstream>

spnav_event sev;
std::ifstream mouse_fd;

/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower() {
    mCopyWorld = false;
    mWorld = NULL;
}

/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower( robotics::World &_world,
                        bool _copyWorld,
			double _configStep ) {

    mCopyWorld = _copyWorld;

    if( mCopyWorld ) {
       printf( "Not implemented yet. Sorry -- achq \n" );
    } else {
        mWorld = &_world;
    }

    mConfigStep = _configStep;
}

/**
 * @function ~JTFollower
 * @brief Destructor
 */
JTFollower::~JTFollower() {

    if( mCopyWorld ) {
        delete mWorld;
    }
}

/**
 * @function init
 */
void JTFollower::init( int _robotId,
		       const Eigen::VectorXi &_links,
		       std::string _EEName,
		       int _EEId,
		       double _res ) {

  mRobotId = _robotId;
  mLinks = _links;

  mMaxIter = 1000;
  mWorkspaceThresh = _res; // An error of half the resolution
  mEENode = (dynamics::BodyNodeDynamics*) mWorld->getRobot(mRobotId)->getNode( _EEName.c_str() );
  mEEId = _EEId;
}

/**
 * @function planPath
 * @brief Main function
 */
std::vector< Eigen::VectorXd > JTFollower::PlanPath( const Eigen::VectorXd &_start,
						     const std::vector<Eigen::VectorXd> &_workspacePath ) {

  //-- Follow the path
  std::vector< Eigen::VectorXd > configPath;
  Eigen::VectorXd q;

  int numPoints = _workspacePath.size();

  //-- Initialize
  q = _start;

  for( size_t i = 1; i < numPoints; ++i ) { // start from 1 since 0 is the current start position
    if( GoToXYZ( q, GetXYZ(_workspacePath[i]), configPath ) == false ) {
      printf(" --(x) An error here, stop following path \n"); break;
    }
  }

  printf("End of Plan Path \n");
  return configPath;

}

/**
 * @function GetPseudoInvJac
 */
Eigen::MatrixXd JTFollower::GetPseudoInvJac( Eigen::VectorXd _q ) {
  printf("Num Dependent DOF minus 6D0F is : %d \n", mEENode->getNumDependentDofs() - 6 );
  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  std::cout<< "Jaclin: \n"<<Jaclin << std::endl;
  Eigen::MatrixXd JaclinT = Jaclin.transpose();
  Eigen::MatrixXd Jt;
  Eigen::MatrixXd JJt = (Jaclin*JaclinT);
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JJt);
  Jt = JaclinT*( lu.inverse() );
  std::cout<< "Jaclin pseudo inverse: \n"<<Jt << std::endl;
  return Jt;
}

/**
 * @function GoToXYZ
 */
bool JTFollower::GoToXYZ( Eigen::VectorXd &_q,
			  Eigen::VectorXd _targetXYZ,
			  std::vector<Eigen::VectorXd> &_workspacePath ) {

  Eigen::VectorXd dXYZ;
  Eigen::VectorXd dConfig;
  int iter;
  mWorld->getRobot(mRobotId)->update();

  //-- Initialize
  dXYZ = ( _targetXYZ - GetXYZ(_q) ); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();
  if(mWorld->checkCollision()) {
    std::cout << "Collision in GetXYZ, blowing up the coords!" << std::endl;
    dXYZ = 100*dXYZ;
  }
  iter = 0;
  //printf("New call to GoToXYZ: dXYZ: %f  \n", dXYZ.norm() );
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    printf("XYZ Error: %f \n", dXYZ.norm() );
    Eigen::MatrixXd Jt = GetPseudoInvJac(_q);
    dConfig = Jt*dXYZ;
    printf("dConfig : %.3f \n", dConfig.norm() );
    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
      printf("NEW dConfig : %.3f \n", dConfig.norm() );
    }
    _q = _q + dConfig;
    _workspacePath.push_back( _q );

    dXYZ = (_targetXYZ - GetXYZ(_q) );
    iter++;
  }

  if( iter >= mMaxIter ) { return false; }
  else { return true; }

}

/**
 * @function GetXYZ
 */
Eigen::VectorXd JTFollower::GetXYZ( Eigen::VectorXd _q ) {


  // Get current XYZ position
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();

  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);



  return qXYZ;
}

bool Open3DMouse() {
  mouse_fd.open("./3d_mouse_input");
  return 0; // No local daemon
  if(spnav_open() == -1) {
    fprintf(stderr, "Failed to connect to space nav daemon\n");
    return 1;
  }
  return 0;
}

bool Close3DMouse() {
  mouse_fd.close();
  return 0;
  spnav_close();
  return 0;
}

Eigen::VectorXd Get3DMouse(Eigen::VectorXd mouse_calibration) {
  std::cout << "Begin" << std::endl;
  // mouse_calibration is a calibration matrix.
  Eigen::VectorXd mouse_translation(3); //ignore rotation for now, so can get by with vector
  mouse_translation << 0,0,0;
  Eigen::VectorXd normalize(3); normalize << 350.0, 350.0, 350.0;

  char mouse_input[256];
  mouse_fd.getline(mouse_input,256);
  std::cout << mouse_input << std::endl;
  std::vector<std::string> strs;
  boost::split(strs, mouse_input, boost::is_any_of("|"));

  std::vector<double> vals;
  for(std::vector<std::string>::iterator it = strs.begin(); it != strs.end(); ++it){
    double num;
    std::stringstream strStream(*it);
    strStream >> num;
    vals.push_back(num);
  }
  double *ptr = &vals[0];
  mouse_translation = Eigen::Map<Eigen::VectorXd>(ptr,3);



  /* // Temporary bypass, use stdin instead!
  while(!spnav_wait_event(&sev)) {
  }
  if(sev.type == SPNAV_EVENT_MOTION) {
    mouse_translation << sev.motion.x, sev.motion.y, sev.motion.z; // make the vector
    mouse_translation = mouse_translation.cwiseQuotient(normalize).cwiseProduct(mouse_calibration);
    std::cout << mouse_translation << std::endl;
    } else {
  }
  */
  std::cout << mouse_translation << std::endl;
  std::cout << "Done" << std::endl;
  return mouse_translation;


}
