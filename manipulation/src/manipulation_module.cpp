#include <manipulation_module.h>

double ManipulationModule::getPeriod()
{
    return 1.0;
}

bool ManipulationModule::updateModule()
{
    if(manipulation->getSensoryInputs())
    {
       if(manipulation->detectMarkersAndComputePose())
       {
           /*image = manipulation->getCVMat();
           if(!image.empty())
           {
               if(manipulation->displayImage(image))
                   stopModule();
           }*/
       } 
    }
    return true;
}

bool ManipulationModule::interruptModule()
{
    return true;
}

bool ManipulationModule::close()
{
    return true;
}
