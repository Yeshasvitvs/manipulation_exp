#include <manipulation_module.h>

double ManipulationModule::getPeriod()
{
    return 1.0;
}

bool ManipulationModule::updateModule()
{
    /*if(manipulation->getSensoryInputs())
    {
       if(manipulation->detectMarkersAndComputePose())
       {
           image = manipulation->getCVMat();
           if(!image.empty())
           {
               if(manipulation->displayImage(image))
                   stopModule();
           }
       }
       else yError() << "Failed to detects markers! Check if the model is loaded correctly in gazebo";
    }*/
    
    if(manipulation->getPoseAndWrenchInput())
    {
        //TODO write them to a text file together
        manipulation->getPoseAndWrenchInfo();
    }
    else{
        yError() << "Error in reading gazebo pose and wrench input values";
        return false;
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
