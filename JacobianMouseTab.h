/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef TUTORIAL_0_TAB
#define TUTORIAL_0_TAB

#include <Tabs/GRIPTab.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>

#include <iostream>
#include <list>

#include "PathPlanner.h"

/**
 * @class TutorialTab
 * @brief Implements simulations for Hubo
 */
class JacobianMouseTab : public GRIPTab
{
public:

  // Constants
  Eigen::VectorXi mLinks;
  Eigen::VectorXd mConfig;
  int mNumLinks;

  int mEEId;
  int mRobotId;
  std::string mEEName;

  bool mContinueRunning;

  // RRT & JT
  Eigen::VectorXd mStartHardcode;
  Eigen::VectorXd mStartConfig;
  Eigen::VectorXd mGoalConfig;
  Eigen::VectorXd mTargetXYZ;

  wxTextCtrl *mTimeText;

  // public vars to capture external selection stuff
  robotics::Object* mSelectedObject;
  robotics::Robot* mSelectedRobot;
  dynamics::BodyNodeDynamics* mSelectedNode;

  wxTextCtrl *mTargetX_Text;
  wxTextCtrl *mTargetY_Text;
  wxTextCtrl *mTargetZ_Text;

  /// Functions

  JacobianMouseTab(){};
  JacobianMouseTab( wxWindow * parent, wxWindowID id = -1,
	       const wxPoint & pos = wxDefaultPosition,
	       const wxSize & size = wxDefaultSize,
	       long style = wxTAB_TRAVERSAL);
  virtual ~JacobianMouseTab(){}

  // Utilities
  void getLinks();
  void printLinks();

  // GUI functions
  void OnButton(wxCommandEvent &evt );
  void OnSlider(wxCommandEvent &evt);
  void SetTimeline( std::list< Eigen::VectorXd > _path , bool resetPath);
  void SetTimeline( std::vector< Eigen::VectorXd > _path, bool resetPath);
  void GRIPStateChange();
  void OnCheckBox( wxCommandEvent &evt);

    // Thread specific
    // GRIPThread* thread;

    // Your Thread routine
    // call GRIPThread::CheckPoint() regularly
    // void Thread();
    // void onCompleteThread();

    DECLARE_DYNAMIC_CLASS( JacobianMouseTab )
    DECLARE_EVENT_TABLE()
};

#endif /** TUTORIAL_0_TAB */
