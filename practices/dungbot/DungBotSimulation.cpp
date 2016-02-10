/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Strøm-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#include "DungBotSimulation.h"

namespace lpzrobots {

DungBotSimulation::DungBotSimulation() {
	setTitle("DungBot simulation");
	setGroundTexture("whiteground_crosses.jpg");
	simulation_time_seconds = 0.0;
	trial_number = 0;
	agent = NULL;
}

DungBotSimulation::~DungBotSimulation() {
	// TODO Auto-generated destructor stub
}

bool DungBotSimulation::command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
	return false;
}

void DungBotSimulation::bindingDescription(osg::ApplicationUsage& au) const {
}

void DungBotSimulation::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) {

	// Configure environment
	setCameraHomePos(Pos(-5, 5, 2),  Pos(-135, -8.5, 0));
	global.odeConfig.setParam("controlinterval", 1);
	global.odeConfig.setParam("gravity", -9.8);

   // Configure simulation
	simulation_time_seconds = 100;
	number_of_runs = 1;
	instantiateAgent(global);
}

void DungBotSimulation::addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
	if (globalData.sim_step >= simulation_time)
		simulation_time_reached=true;
}

bool DungBotSimulation::restart(const OdeHandle&, const OsgHandle&, GlobalData& global)
{
	if (this->currentCycle == number_of_runs)
		return false;

	if (agent!=0) {
		OdeAgentList::iterator itr = find(global.agents.begin(),global.agents.end(),agent);
		if (itr!=global.agents.end())
		{
			global.agents.erase(itr);
		}
		delete agent;
		agent = 0;
	}

	instantiateAgent(global);

	return true;
}

void DungBotSimulation::setSimulationDuration(double seconds) {
	simulation_time = (long)(seconds/0.01);
}

void DungBotSimulation::instantiateAgent(GlobalData& global) {
	// Instantiate robot
	LocoKitConf conf = DungBot::getDefaultConf();
	robot = new DungBot(odeHandle, osgHandle, conf, "LocoKit robot");
	robot->place(Pos(.0, .0, -0.2));

	// Instantiate controller
	controller = new DungBotEmptyController("DungBot Controller");

	// Create the wiring
	auto wiring = new One2OneWiring(new NoNoise());

	// Create Agent
	agent = new OdeAgent(global);
	agent->init(controller, robot, wiring);
	global.agents.push_back(agent);
	global.configs.push_back(agent);

	setSimulationDuration( simulation_time_seconds );
}

} /* namespace lpzrobots */


