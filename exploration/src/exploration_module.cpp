#include <exploration_module.h>

double ExplorationModule::getPeriod()
{
    return 0.001; //This is in seconds
}

bool ExplorationModule::updateModule()
{ 
    yInfo() << "Exploration Module Update";
    return true;
}

bool ExplorationModule::interruptModule()
{
    return true;
}

bool ExplorationModule::close()
{
    return true;
}
