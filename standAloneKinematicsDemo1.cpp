#include "ik.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "7Vector.h"

extern "C" {
    #include "extApi.h"
}

// For double-precision, define IK_DOUBLE in the project settings

void prepareShapeNames(std::vector<std::string>& shapeNames)
{   // Prepare a vector with all the shape names in the robot (except for the fixed shape, i.e. the base shape)).
    // The names in the exported IK file and the names in the CoppeliaSim scene should be the same:
    shapeNames.clear();
    shapeNames.push_back("irb360_arm");
    shapeNames.push_back("irb360_arm0");
    shapeNames.push_back("irb360_arm1");
    shapeNames.push_back("irb360_axisL");
    shapeNames.push_back("irb360_axisL0");
    shapeNames.push_back("irb360_axisL1");
    shapeNames.push_back("irb360_axisL2");
    shapeNames.push_back("irb360_bridgeB");
    shapeNames.push_back("irb360_bridgeB0");
    shapeNames.push_back("irb360_bridgeB1");
    shapeNames.push_back("irb360_bridgeT");
    shapeNames.push_back("irb360_bridgeT0");
    shapeNames.push_back("irb360_bridgeT1");
    shapeNames.push_back("irb360_linkL");
    shapeNames.push_back("irb360_linkL0");
    shapeNames.push_back("irb360_linkL1");
    shapeNames.push_back("irb360_linkL2");
    shapeNames.push_back("irb360_linkL3");
    shapeNames.push_back("irb360_linkL4");
    shapeNames.push_back("irb360_platform");
}

int getSimHandles(int clientID,const std::vector<std::string>& shapeNames,std::vector<int>& simShapeHandles,const char* baseName)
{ // Here we retrieve info from the CoppeliaSim scene: the handles of the shape named in 'shapeNames', and the baseObjectHandle:
    simShapeHandles.clear();

    // 1. We retrieve the handles and names of all shapes in the current CoppeliaSim scene:
    int _allSimShapeCnt,_tmp;
    int* _allSimShapeHandlesTmp;
    char* _allSimShapeNamesTmp;
    simxGetObjectGroupData(clientID,sim_object_shape_type,0,&_allSimShapeCnt,&_allSimShapeHandlesTmp,NULL,NULL,NULL,NULL,&_tmp,&_allSimShapeNamesTmp,simx_opmode_blocking);

    // 2.a. We put all CoppeliaSim shape handles and names into 2 corresponding vectors, for easier parsing:
    std::vector<int> _allSimShapeHandles_forParsing;
    std::vector<std::string> _allSimShapeNames_forParsing;
    // 2.b. We fill the two vectors:
    size_t off=0;
    for (int i=0;i<_allSimShapeCnt;i++)
    {
        _allSimShapeNames_forParsing.push_back(_allSimShapeNamesTmp+off);
        _allSimShapeHandles_forParsing.push_back(_allSimShapeHandlesTmp[i]);
        off+=strlen(_allSimShapeNamesTmp+off)+1;
    }

    // 3. We now go through all robot shapes and insert the corresponding CoppeliaSim object handle:
    for (size_t i=0;i<shapeNames.size();i++)
    {
        int handle=-1;
        for (size_t j=0;j<_allSimShapeNames_forParsing.size();j++)
        {
            if (_allSimShapeNames_forParsing[j].compare(shapeNames[i])==0)
            {
                handle=_allSimShapeHandles_forParsing[j];
                break;
            }
        }
        if (handle!=-1)
            simShapeHandles.push_back(handle);
    }

    // Now get the handle of the robot's base:
    int baseHandle;
    simxGetObjectHandle(clientID,baseName,&baseHandle,simx_opmode_blocking);

    return(baseHandle);
}

void switchToFk(const int* motorHandles,int mainIkGroup,int tipHandle)
{   // Here we switch the robot to FK mode.

    // 1. We want the robot motors NOT to be part of IK resolution:
    ikSetJointMode(motorHandles[0],ik_jointmode_passive);
    ikSetJointMode(motorHandles[1],ik_jointmode_passive);
    ikSetJointMode(motorHandles[2],ik_jointmode_passive);

    // 2. We disable the IK element that handles the position of the tip (since we are in FK mode):
    ikSetIkElementConstraints(mainIkGroup,tipHandle|ik_handleflag_tipdummy,0); // i.e. no constraints for that element
}

void switchToIk(const int* motorHandles,int mainIkGroup,int tipHandle,int targetHandle)
{   // Here we switch the robot to IK mode.

    // 1. We want the robot motors to be part of IK resolution:
    ikSetJointMode(motorHandles[0],ik_jointmode_ik);
    ikSetJointMode(motorHandles[1],ik_jointmode_ik);
    ikSetJointMode(motorHandles[2],ik_jointmode_ik);

    // 2. We enable the IK element that handles the position of the tip (since we are in IK mode):
    ikSetIkElementConstraints(mainIkGroup,tipHandle|ik_handleflag_tipdummy,ik_constraint_position); // i.e. that element is constrained in x/y/z position

    // 3. We make sure the target is at the same position and orientation as the tip:
    C7Vector identityTransf;
    identityTransf.setIdentity();
    ikSetObjectTransformation(targetHandle,tipHandle,&identityTransf);
}

void readShapeTransformationsFromKinematicsRoutinesAndApplyToSim(int clientID,const std::vector<int>& shapeHandles,int baseHandle,const std::vector<int>& simShapeHandles,int simBaseHandle,const std::vector<C7Vector>& corrTransfs)
{
    simxPauseCommunication(clientID,1); // we temporarily pause the communication with CoppeliaSim, so that all positions/orientations are set at the same time on the CoppeliaSim side

    for (size_t i=0;i<shapeHandles.size();i++)
    { // we skip the robot base. But we read/write everything relative to the robot base!
        C7Vector transf;
        ikGetObjectTransformation(shapeHandles[i],baseHandle,&transf);
        C7Vector tr(transf*corrTransfs[i]);
        C3Vector euler(tr.Q.getEulerAngles());
        float posf[3];
        float eulerf[3];
        for (size_t j=0;j<3;j++)
        { // the Coppelia Kinematics Routines can use double precision!
            posf[j]=(float)tr.X(j);
            eulerf[j]=(float)euler(j);
        }
        simxSetObjectPosition(clientID,simShapeHandles[i],simBaseHandle,posf,simx_opmode_oneshot);
        simxSetObjectOrientation(clientID,simShapeHandles[i],simBaseHandle,eulerf,simx_opmode_oneshot);
    }
    simxPauseCommunication(clientID,0);
}

void getCorrectionTransformations(int clientID,const std::vector<int>& shapeHandles,int baseHandle,const std::vector<int>& simShapeHandles,int simBaseHandle,std::vector<C7Vector>& corrTransfs)
{ // Here we compute: corrTransfs=inv(transf)*tr. This is the correction transformation.
    // Later, when we set the transformation to CoppeliaSim, we simply need to multiply the transformation from the external IK with that correction
    // There is a correction transformation for each shape.
    corrTransfs.clear();
    for (size_t i=0;i<shapeHandles.size();i++)
    {
        C7Vector transf;
        ikGetObjectTransformation(shapeHandles[i],baseHandle,&transf);
        float simPosf[3];
        float simEulerf[3];
        simxGetObjectPosition(clientID,simShapeHandles[i],simBaseHandle,simPosf,simx_opmode_blocking);
        simxGetObjectOrientation(clientID,simShapeHandles[i],simBaseHandle,simEulerf,simx_opmode_blocking);
        simReal simPos[3];
        simReal simEuler[3];
        for (size_t j=0;j<3;j++)
        { // the Coppelia Kinematics Routines can use double precision!
            simPos[j]=(simReal)simPosf[j];
            simEuler[j]=(simReal)simEulerf[j];
        }
        C7Vector tr;
        tr.X=C3Vector(simPos);
        tr.Q=C4Vector(C3Vector(simEuler));
        C7Vector corrTr(transf.getInverse()*tr);
        corrTransfs.push_back(corrTr);
    }
}

int main(int argc, char* argv[])
{
    // Retrieve the command line argument that represents the port where the remote API should connect:
    int portNb=0;
    if (argc>=2)
        portNb=atoi(argv[1]);
    else
    {
        printf("Indicate following arguments: 'portNumber'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Prepare a vector with all the shape names in the robot (except for the fixed shape in the model).
    // The names in the exported IK file and the names in the CoppeliaSim scene should be the same:
    std::vector<std::string> shapeNames;
    prepareShapeNames(shapeNames);

    // Read the IK file for that robot (the names should be the same as in the CoppeliaSim scene used to visualize the robot movement)
    FILE *file;
    file=fopen("irb360s.ik","rb");
    unsigned char* data=NULL;
    int dataLength=0;
    if (file)
    {
        fseek(file,0,SEEK_END);
        unsigned long fl=ftell(file);
        dataLength=(int)fl;
        fseek(file,0,SEEK_SET);
        data=new unsigned char[dataLength];
        fread((char*)data,dataLength,1,file);
        fclose(file);
    }
    else
    {
        printf("The kinematic content file 'irb360s.ik' could not be read!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Initialize the embedded robot model in the external IK:
    ikCreateEnvironment();
    ikLoad(data,dataLength);
    delete[] data;

    // Now retrieve a few handles FROM THE EXPORTED IK FILE:
    int motorHandles[4]; // the robot's motor handles
    ikGetObjectHandle("irb360_drivingJoint1",motorHandles+0); // the motor of arm1
    ikGetObjectHandle("irb360_drivingJoint2",motorHandles+1); // the motor of arm2
    ikGetObjectHandle("irb360_drivingJoint3",motorHandles+2); // the motor of arm3
    ikGetObjectHandle("irb360_motor",motorHandles+3); // the central motor (i.e. for the orientation)
    int baseHandle;
    ikGetObjectHandle("irb360_base",&baseHandle); // the base object relative to which we do retrieve matrices
    int tipHandle;
    ikGetObjectHandle("irb360_ikTip",&tipHandle); // the tip object
    int targetHandle;
    ikGetObjectHandle("irb360_ikTarget",&targetHandle); // the target object
    int mainIkGroup;
    ikGetIkGroupHandle("irb360_mainTask",&mainIkGroup); // the main IK handle. This is needed when we switch from IK to FK and vice-versa

    std::vector<int> shapeHandles; // all the shape handles. Those are needed in order to reflect the position/orientation of the shapes of the robot in the CoppeliaSim scene
    for (size_t i=0;i<shapeNames.size();i++)
    {
        int h;
        ikGetObjectHandle(shapeNames[i].c_str(),&h);
        shapeHandles.push_back(h);
    }

    // Now connect to CoppeliaSim:
    int clientID=simxStart("127.0.0.1",portNb,true,true,2000,5);
    if (clientID!=-1)
    { // connection was successful

        float simulationStep;
        simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&simulationStep,simx_opmode_streaming);

        simxSynchronous(clientID,1); // We enable the synchronous mode, so that we can trigger each simulation step from here

        // Now retrieve the shape handles FROM THE CoppeliaSim SCENE:
        std::vector<int> simShapeHandles; // all CoppeliaSim scene handles that correspond to the shapeNames vector
        int simBaseHandle=getSimHandles(clientID,shapeNames,simShapeHandles,"irb360_base");

        // Now retrieve all the correction matrices that link the reference frame of a given shape in the Coppelia Kinematics Routines,
        // to its corresponding matrix in CoppeliaSim. Correction matrices are not needed when the shapes initially overlap.
        std::vector<C7Vector> corrTransformations;
        getCorrectionTransformations(clientID,shapeHandles,baseHandle,simShapeHandles,simBaseHandle,corrTransformations);

        // Switch to FK mode:
        switchToFk(motorHandles,mainIkGroup,tipHandle);

        simReal v=0.0;
        simReal a=0.0;
        simReal rot=0.0;

        bool breakOut=false;
        for (size_t kk=0;kk<3;kk++)
        {
            for (size_t fkc=0;fkc<60;fkc++)
            {
                // Set the desired motor position:
                ikSetJointPosition(motorHandles[kk],simReal(-45.0*3.1415*sin(float(fkc)*3.1415/60.0)/180.0));
                rot+=simReal(0.01);
                ikSetJointPosition(motorHandles[3],rot);

                // calculate FK:
                ikHandleIkGroup(ik_handle_all);

                // Now we could read the end-effector position relative to the robot base:
                C7Vector tipTransformation;
                ikGetObjectTransformation(tipHandle,baseHandle,&tipTransformation);

                // Read and apply all shape positions:
                readShapeTransformationsFromKinematicsRoutinesAndApplyToSim(clientID,shapeHandles,baseHandle,simShapeHandles,simBaseHandle,corrTransformations);

                // Trigger next simulation step:
                int r=simx_return_remote_error_flag; // means for next remote API function call: step not triggered
                while (r==simx_return_remote_error_flag)
                    r=simxSynchronousTrigger(clientID); // Trigger next simulation step
                if (r!=simx_return_ok)
                {
                    breakOut=true;
                    break;
                }

                printf(".");
            }
            if (breakOut)
                break;
        }

        // Switch to IK mode:
        switchToIk(motorHandles,mainIkGroup,tipHandle,targetHandle);

        C7Vector initialTransformation;
        ikGetObjectTransformation(tipHandle,baseHandle,&initialTransformation); // get the transformation relative to the robot base

        while (simxGetConnectionId(clientID)!=-1)
        {
            // Following 3 commands will slow down the simulation, but garantee that if the simulation time step was changed,
            // that there won't be any jumps. Following 3 commands are not needed if you don't modify the simulation time step
            // (i.e. dt) during simulation.
            simxUChar simWaitingForTrigger=0;
            while ( (simWaitingForTrigger==0)&&(simxGetConnectionId(clientID)!=-1) )
                simxGetBooleanParameter(clientID,sim_boolparam_waiting_for_trigger,&simWaitingForTrigger,simx_opmode_blocking);

            simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&simulationStep,simx_opmode_buffer);
            v=v+simReal(0.08*simulationStep*20.0);
            if (v>simReal(10.0))
            {
                a=a+simReal(0.0005);
                if (a>1.0)
                    a=1.0;
            }
            rot+=simReal(0.01*simulationStep*20.0);

            // Set the desired tip position:
            simReal x=simReal(sin(v)*0.5);
            simReal y=simReal(cos(v)*0.5);
            simReal z=simReal(sin(v*3.0)*0.1);

            C7Vector newTransformation(initialTransformation);
            newTransformation.X+=C3Vector(simReal(x*a*1.1),simReal(y*a*1.1),simReal(z*(1.0-a)));
            ikSetObjectTransformation(targetHandle,baseHandle,&newTransformation); // set the transformation relative to the robot base

            // Set the desired central motor rotation:
            ikSetJointPosition(motorHandles[3],rot);

            // calculate IK:
            ikHandleIkGroup(ik_handle_all);

            // Now we could read the computed motor angles:
            simReal motorAngles[3];
            ikGetJointPosition(motorHandles[0],motorAngles);
            ikGetJointPosition(motorHandles[1],motorAngles+1);
            ikGetJointPosition(motorHandles[2],motorAngles+2);

            // Read and apply all shape positions:
            readShapeTransformationsFromKinematicsRoutinesAndApplyToSim(clientID,shapeHandles,baseHandle,simShapeHandles,simBaseHandle,corrTransformations);

            // Trigger next simulation step:
            int r=simx_return_remote_error_flag; // means for next remote API function call: step not triggered
            while (r==simx_return_remote_error_flag)
                r=simxSynchronousTrigger(clientID); // Trigger next simulation step
            if (r!=simx_return_ok)
                break;

            printf(".");
        }
        simxFinish(clientID);
    }
    ikEraseEnvironment();
    return(0);
}

