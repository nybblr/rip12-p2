/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "GRIPApp.h"
#include "TutorialTab.h"

extern wxNotebook* tabView;

/**
	* @class TutorialTabApp
	*/
class TutorialTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new TutorialTab(tabView), wxT("TutorialTab"));
	}
};

IMPLEMENT_APP(TutorialTabApp)
