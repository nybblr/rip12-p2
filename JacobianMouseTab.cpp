/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "JacobianMouseTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <GRIPApp.h>

#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <iostream>

#include "JTFollower/JTFollower.h"


// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum JacobianMouseTabEvents {
	button_Set_Start,
	button_Set_Hardcode_Start,
	button_Set_Goal,
	button_Show_Start,
	button_Show_Goal,
	button_Check_Collision,
	button_Run,
	checkbox_ContinueRunning
};

// sizer for whole tab
wxBoxSizer* sizerFullJacobian;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE(JacobianMouseTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, JacobianMouseTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, JacobianMouseTab::OnSlider)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, JacobianMouseTab::OnCheckBox)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(JacobianMouseTab, GRIPTab)

/**
 * @function JacobianMouseTab
 * @brief Constructor
 */
JacobianMouseTab::JacobianMouseTab( wxWindow *parent, const wxWindowID id,
			  const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {

  mRobotId = 0;
  mContinueRunning = false;

  sizerFullJacobian = new wxBoxSizer( wxHORIZONTAL );

  // ** Create left static box for Configure **
  wxStaticBox* configureSBox = new wxStaticBox(this, -1, wxT("Configuration"));

  // Create sizer for this box
  wxStaticBoxSizer* configureSBoxSizer = new wxStaticBoxSizer(configureSBox, wxHORIZONTAL);

  // Start settings
  wxBoxSizer* startBoxSizer = new wxBoxSizer( wxVERTICAL );

  configureSBoxSizer->Add( startBoxSizer, 1, wxALIGN_NOT, 0 );

  startBoxSizer->Add( new wxButton(this, button_Set_Start, wxT("Set Start")),
			  0,  wxALL,  1 );
  startBoxSizer->Add( new wxButton(this, button_Show_Start, wxT("Show Start")),
			  0,  wxALL,  1 );
  startBoxSizer->Add( new wxButton(this, button_Set_Hardcode_Start, wxT("Set hard-code Start")),
			  0,  wxALL,  1 );

  // Goal settings
  wxBoxSizer* goalBoxSizer = new wxBoxSizer( wxVERTICAL );

  configureSBoxSizer->Add( goalBoxSizer, 1, wxALIGN_NOT, 0 );

  goalBoxSizer->Add( new wxButton(this, button_Set_Goal, wxT("Set Goal")),
		     0,  wxALL,  1 );

  // ** Create sizer for Target Position inputs **
  wxBoxSizer *PosInputSizer = new wxBoxSizer(wxVERTICAL);

  // x
  wxBoxSizer *posX_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posX_Label = new wxStaticText( this, 1007, wxT("x: ") );
  mTargetX_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(60,20),wxTE_LEFT);

  posX_Sizer->Add( posX_Label, 0, wxALL, 1 );
  posX_Sizer->Add( mTargetX_Text, 0, wxALL, 1 );

  PosInputSizer->Add( posX_Sizer, 0, wxALL, 1 );

  // y
  wxBoxSizer *posY_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posY_Label = new wxStaticText( this, 1007, wxT("y: ") );
  mTargetY_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(60,20),wxTE_LEFT);

  posY_Sizer->Add( posY_Label, 0, wxALL, 1 );
  posY_Sizer->Add( mTargetY_Text, 0, wxALL, 1 );

  PosInputSizer->Add( posY_Sizer, 0, wxALL, 1 );

  // z
  wxBoxSizer *posZ_Sizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *posZ_Label = new wxStaticText( this, 1007, wxT("z: ") );
  mTargetZ_Text = new wxTextCtrl(this,1008,wxT("0.0"), wxDefaultPosition,wxSize(60,20),wxTE_LEFT);

  posZ_Sizer->Add( posZ_Label, 0, wxALL, 1 );
  posZ_Sizer->Add( mTargetZ_Text, 0, wxALL, 1 );

  PosInputSizer->Add( posZ_Sizer, 0, wxALL, 1 );

  goalBoxSizer->Add( PosInputSizer, 0,  wxALL,  1 );

  goalBoxSizer->Add( new wxButton(this, button_Show_Goal, wxT("Show Goal")),
		     0,  wxALL,  1 );

  sizerFullJacobian->Add(configureSBoxSizer, 3, wxEXPAND | wxALL, 6);


    // ** Create right static box for experiment1Box **
    wxStaticBox* runSBox = new wxStaticBox(this, -1, wxT("Run"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* runBoxSSizer = new wxStaticBoxSizer( runSBox, wxVERTICAL );

    // Create sizer for Experiment 1 buttons in 2nd column
    wxBoxSizer *runBoxSizer = new wxBoxSizer(wxVERTICAL);
    runBoxSizer->Add( new wxButton(this, button_Check_Collision, wxT("Check Collision")),
		      0,
		      wxALL, 1 );
    runBoxSizer->Add( new wxButton(this, button_Run, wxT("Run")),
		      0, wxALL, 1 );
    runBoxSizer->Add( new wxCheckBox(this, checkbox_ContinueRunning, _T("Keep running (live update mode)")), 1, wxALL, 0);


    // Add col1Sizer to the configuration box
    runBoxSSizer->Add( runBoxSizer, 1, wxALIGN_NOT );

    // Add this box to parent sizer
    sizerFullJacobian->Add( runBoxSSizer, 3,
		    wxEXPAND | wxALL, 6 );


    SetSizer(sizerFullJacobian);

}


/**
 * @function getLinks
 */
void JacobianMouseTab::getLinks() {

  mNumLinks = mWorld->getRobot(mRobotId)->getNumQuickDofs();

  mLinks.resize( mNumLinks );
  mLinks =  mWorld->getRobot(mRobotId)->getQuickDofsIndices();
  int EEDofId = mLinks( mNumLinks - 1 );
  mEEId = mWorld->getRobot(mRobotId)->getDof( EEDofId )->getJoint()->getChildNode()->getSkelIndex();
  mEEName =  mWorld->getRobot(mRobotId)->getNode(mEEId)->getName();
  std::cout << "Link IDs: " << mLinks.transpose() << std::endl;
  std::cout << " EE Name: "<<mEEName << std::endl;

  // Only for Schunk (no hand) -- Comment otherwise
  mStartHardcode.resize( mNumLinks );
  mStartHardcode << 0.528,  -1.089,  0.176,  -1.156,  -0.276,  -0.873,  0.000; // -> Pointing down
 // mStartHardcode <<  0.075,  -1.022,  -0.478,  -1.022,  0.000,  -0.854,  0.000 ;

}

/**
 * @function printLinks
 */
void JacobianMouseTab::printLinks() {

  //-- Right Arm
  printf( "* Num Links: %d \n", mNumLinks );
  for( int i = 0; i < mNumLinks; ++i ) {
    printf(" %d ", mLinks[i] );
  }
  printf("\n");
}

void JacobianMouseTab::OnCheckBox( wxCommandEvent &evt) {
  int checkbox_num = evt.GetId();

  switch (checkbox_num) {
  case checkbox_ContinueRunning:
    mContinueRunning = (bool)evt.GetSelection();
    std::cout << "(i) ContinueRunning = " << mContinueRunning << std::endl;
    break;
  }
}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void JacobianMouseTab::OnButton(wxCommandEvent &evt) {

  int button_num = evt.GetId();
  getLinks();

  switch (button_num) {

    /** Set Start */
  case button_Set_Start:
    if ( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1) {
	printf( "(!) Must have a world with a robot to set a Start state" );
	break;
      }
      mStartConfig.resize(0);
      mStartConfig = mWorld->getRobot(mRobotId)->getDofs( mLinks );

      printf("* Start Configuration: ");
      for( int i = 0; i< mStartConfig.size(); i++ )
	{  printf( " %.3f ",  mStartConfig(i) ); }
      printf("\n");

    } else {
      printf("(!) Must have a world loaded to set a Start state \n");
    }
    break;

    /** Set Hard-code Start */
  case button_Set_Hardcode_Start:
    if ( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1) {
	printf( "(!) Must have a world with a robot to set a Start state" );
	break;
      }
      mStartConfig.resize(0);
      mStartConfig = mStartHardcode;

      printf("* Start Configuration: ");
      for( int i = 0; i< mStartConfig.size(); i++ )
	{  printf( " %.3f ",  mStartConfig(i) ); }
      printf("\n");

    } else {
      printf("(!) Must have a world loaded to set a Start state \n");
    }
    break;


    /** Show Start */
  case button_Show_Start:
    if( mStartConfig.size() < 1 ){
      printf( "(!) First, set a start configuration \n");
      break;
    }

    printf("Start Configuration: ");
    mWorld->getRobot(mRobotId)->setDofs( mStartConfig, mLinks );

    for( int i = 0; i< mStartConfig.size(); i++ )
      {  printf( " %.3f ",  mStartConfig(i) ); }
    printf("\n");

    mWorld->getRobot(mRobotId)->update();
    viewer->UpdateCamera();
    break;

    /** Set goal */
  case button_Set_Goal:
    if( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1 )
	{  printf("(x) No robot in the loaded world! \n"); break; }

      double x; double y; double z;
      mTargetX_Text->GetValue().ToDouble(&x);
      mTargetY_Text->GetValue().ToDouble(&y);
      mTargetZ_Text->GetValue().ToDouble(&z);

      mTargetXYZ.resize(3);
      //mTargetXYZ << x, y, z;
      Open3DMouse();
      Eigen::VectorXd cal(3); cal << 1,1,1;
      mTargetXYZ = Get3DMouse(cal);
      Close3DMouse();
      printf("** Goal: (%f %f %f) \n", mTargetXYZ(0), mTargetXYZ(1), mTargetXYZ(2) );
    }
    else
      { printf("(x) No world loaded, I cannot set a goal \n"); }
    break;

    /** Show Goal */
  case button_Show_Goal:
    if( mTargetXYZ.size() !=  3 ){
      std::cout << "(x) First, set a goal position" << std::endl;
      break;
    }
    printf("** Goal: (%f %f %f) \n", mTargetXYZ(0), mTargetXYZ(1), mTargetXYZ(2) );

    break;

    /** Check Collisions  */
  case button_Check_Collision:
    std::cout << "(0) Checking Collisions" << std::endl;
    bool st;
    st = mWorld->checkCollision();
    if( st == true )
      { printf("Collisions \n"); }
    else
      { printf("No Collisions \n"); }

    break;


    /** Run */
  case button_Run:
    int num_loops = 0;
    if ( mContinueRunning ){
      num_loops = 0;
    }
    do{
      if ( mContinueRunning ) {
	Open3DMouse();
	Eigen::VectorXd cal(3); cal << 1,1,1;
	mTargetXYZ = Get3DMouse(cal);
	Close3DMouse();
	printf("** Goal: (%f %f %f) \n", mTargetXYZ(0), mTargetXYZ(1), mTargetXYZ(2) );
      }

      printf("Run JT Following \n");
      JTFollower *jt = new JTFollower(*mWorld);
      jt->init( mRobotId, mLinks, mEEName, mEEId, 0.02 );

      PathPlanner *pp = new PathPlanner (*mWorld, false, 0.02);

      std::vector<Eigen::VectorXd> wsPath;
      Eigen::VectorXd start = mStartConfig;

      if( mContinueRunning ) {  // In accumulate mode
	Eigen::VectorXd current = mWorld->getRobot(mRobotId)->getDofs( mLinks ); // current arm position

	 if( jt->GoToXYZ( current, mTargetXYZ, wsPath ) == true){
	   // We have a workspace path now. Lets use it.
	   printf("Found solution JT! Adding to timeline\n");

	   SetTimeline( jt->PlanPath(current, wsPath), false);
	   //SetTimeline( wsPath, false);
	 }
	 else{
	   printf("NO Found solution JT! Plotting anyway \n");

	   SetTimeline(jt->PlanPath(current,wsPath), false);
	   //SetTimeline( wsPath, false);
	 }
      }
      else{
	if( jt->GoToXYZ( start, mTargetXYZ, wsPath ) == true){
	  printf("Found solution JT! \n");
	  SetTimeline( wsPath ,true);
	}
	else{
	  printf("NO Found solution JT! Plotting anyway \n");
	  SetTimeline( wsPath ,true);
	}
      }

    } while ( --num_loops >= 0 );

    break;

  } // end switch
}

/**
 * @function setTimeLine
 * @brief
 */
	void JacobianMouseTab::SetTimeline( std::list< Eigen::VectorXd > _path , bool resetPath) {

  if( mWorld == NULL  ) {
    printf("--(!) Must create a valid plan before updating its duration (!)--");
    return;
  }

  double T = 10;
  int numsteps = _path.size();
  double increment = T/(double)numsteps;

  printf( "** Ready to see Plan: Updated Timeline - Increment: %f, Total T: %f  Steps: %d \n", increment, T, numsteps);

  if(resetPath)
    frame->InitTimer( string("RRT_Plan"),increment );
  Eigen::VectorXd vals( mLinks.size() );

  for( std::list<Eigen::VectorXd>::iterator it = _path.begin(); it != _path.end(); it++ ) {
    mWorld->getRobot( mRobotId )->setDofs( *it, mLinks );
    mWorld->getRobot(mRobotId)->update();
    frame->AddWorld( mWorld );
  }

}

/**
 * @function setTimeLine
 * @brief
 */
 void JacobianMouseTab::SetTimeline( std::vector< Eigen::VectorXd > _path, bool resetPath) {

  if( mWorld == NULL  ) {
    printf("--(!) Must create a valid plan before updating its duration (!)--");
    return;
  }

  double T = 10;
  int numsteps = _path.size();
  double increment = T/(double)numsteps;

  printf( "** Ready to see Plan: Updated Timeline - Increment: %f, Total T: %f  Steps: %d \n", increment, T, numsteps);

  if(resetPath)
    frame->InitTimer( string("Plan"),increment );
  Eigen::VectorXd vals( mLinks.size() );

  for( int i = 0; i < _path.size(); i++ ) {
    mWorld->getRobot( mRobotId )->setDofs( _path[i], mLinks );
    mWorld->getRobot(mRobotId)->update();
    frame->AddWorld( mWorld );
  }

}

/**
 * @function
 * @brief Handle slider changes
 */
void JacobianMouseTab::OnSlider(wxCommandEvent &evt) {

  if(selectedTreeNode==NULL){
    return;
  }

  int slnum = evt.GetId();
  double pos = *(double*) evt.GetClientData();

}


/**
 * @function GRIPStateChange
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void JacobianMouseTab::GRIPStateChange() {
  if ( selectedTreeNode == NULL ) {
    return;
  }

  std::string statusBuf;
  std::string buf, buf2;

  switch (selectedTreeNode->dType) {

  case Return_Type_Object:
    mSelectedObject = (robotics::Object*) ( selectedTreeNode->data );
    statusBuf = " Selected Object: " + mSelectedObject->getName();
    buf = "You clicked on object: " + mSelectedObject->getName();

    // Enter action for object select events here:

    break;
  case Return_Type_Robot:
    mSelectedRobot = (robotics::Robot*) ( selectedTreeNode->data );
    statusBuf = " Selected Robot: " + mSelectedRobot->getName();
    buf = " You clicked on robot: " + mSelectedRobot->getName();

    // Enter action for Robot select events here:

    break;
  case Return_Type_Node:
    mSelectedNode = (dynamics::BodyNodeDynamics*) ( selectedTreeNode->data );
    statusBuf = " Selected Body Node: " + string(mSelectedNode->getName()) + " of Robot: "
      + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();
    buf = " Node: " + std::string(mSelectedNode->getName()) + " of Robot: " + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();

    // Enter action for link select events here:

    break;
  default:
    fprintf(stderr, "--( :D ) Someone else's problem!\n");
    assert(0);
    exit(1);
  }

  //cout << buf << endl;
  frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  sizerFullJacobian->Layout();
}
