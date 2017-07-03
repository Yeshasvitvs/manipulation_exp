#include <exploration_module.h>

double ExplorationModule::getPeriod()
{
    return 5; //This is in seconds
}

bool ExplorationModule::updateModule()
{ 
    exploration->explore();
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
