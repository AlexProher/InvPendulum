// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChSocketCommunication.h"
#include "sourceFiles/MyCart.h"


#include "fstream"
#include "iostream"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::utils;




int main(int argc, char* argv[]) {

    bool control = false;

    try {

        ChSystemNSC sys;
        ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
        sys.SetCollisionSystemType(collision_type);

        // Create a Chrono physical system
        MyCart cart(ChVector3d(0, 1.5f, 0));
        cart.addCartToSys(sys);

        // 1 - Create a floor that is fixed (that is used also to represent the absolute reference)

        double xFloorSize = 100;
        double yFloorSize = 1;
        double zFloorSize = 100;
        double mass = 0.1f;


        auto matVisDefault = chrono_types::make_shared<ChVisualShapeBox>(xFloorSize, yFloorSize, zFloorSize);
        matVisDefault->SetColor(ChColor(0.1f, 0.1f, 0.1f));

        auto defMat = chrono_types::make_shared<ChContactMaterialNSC>();
        auto collshape = chrono_types::make_shared<ChCollisionShapeBox>(defMat, xFloorSize, yFloorSize, zFloorSize);

        auto floorBody = chrono_types::make_shared<ChBody>();
        //floorBody->SetMass(mass);

        //floorBody->SetInertiaXX(ChVector3d((1.0 / 12.0) * mass * (pow(yFloorSize, 2) + pow(zFloorSize, 2)),
        //                        (1.0 / 12.0) * mass * (pow(xFloorSize, 2) + pow(zFloorSize, 2)),
        //                        (1.0 / 12.0) * mass * (pow(xFloorSize, 2) + pow(yFloorSize, 2))));

        floorBody->AddCollisionShape(collshape);
        floorBody->AddVisualShape(matVisDefault);
        floorBody->EnableCollision(true);
        floorBody->SetFixed(true);

        sys.Add(floorBody);

        sys.SetGravitationalAcceleration(ChVector3d(0, -9.8f, 0));
        sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys.GetSolver()->AsIterative()->SetMaxIterations(20);

        // Add a socket framework object
        ChSocketFramework socket_tools;

        //// Create the cosimulation interface:

        ChSocketCommunication cosimul_interface(socket_tools,
            1,   // n.input values from Simulink
            4);  // n.output values to Simulink

        if (control) {

            //// 4) Wait client (Simulink) to connect...
            std::cout << " *** Waiting Simulink to start... ***\n"
                << "(load 'data/cosimulation/test_cosim_hydraulics.mdl' in Simulink and press Start...)\n"
                << std::endl;

            int PORT_NUMBER = 50009;

            cosimul_interface.WaitConnection(PORT_NUMBER);
        }
        

        // Prepare the two column vectors of data that will be swapped
        // back and forth between Chrono and Simulink. In detail we will
        // - receive 1 variable from Simulink (the hydraulic cylinder force)
        // - send 2 variables to Simulink (the hydraulic cylinder velocity and displacement)
        ChVectorDynamic<double> data_in(1);
        ChVectorDynamic<double> data_out(4);
        data_in.setZero();
        data_out.setZero();

        double mytime = 0;
        double histime = 0;

        //// Here the 'dt' must be the same of the sampling period that is
        //// entered in the CEcosimulation block

        double dt = 0.001;

        // Optionally, set color and/or texture for visual assets

        // 4 - Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("A simple pendulum example");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 14, -20));
        vis->AddTypicalLights();

        ChRealtimeStepTimer realtime_timer;

        double time = 0;

        std::ofstream myfile("example.txt");
        if (!myfile.is_open()) {
            std::cout << "Unable to open file";
            return 0;
        }

        myfile << "time\tx_pos\tx_vel\tx_angle\tdx_angle\n";

        while (vis->Run()) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            // Perform the integration stpe
            sys.DoStepDynamics(dt);
            time += dt;


            data_out(3) = -cart.getBodyVel().x();
            data_out(2) = -cart.getBodyPos().x();
            data_out(1) = cart.getSphereAngleDt().z();
            data_out(0) = cart.getSphereAngle().z();
            //std::cout << "--- vel: " << data_out(3)
            //        << "--- pos: " << data_out(2)
            //        << "--- angle_dX: " << data_out(1)
            //        << "--- angle_X: " << data_out(0) << std::endl;



            // Spin in place to maintain soft real-time
            //realtime_timer.Spin(dt);
            //myfile << time;
            //myfile << '\t';
            //myfile << data_out(2);
            //myfile << '\t';
            //myfile << data_out(3);
            //myfile << '\n';
            //myfile << data_out(0);
            //myfile << '\n';
            //myfile << data_out(1);
            //myfile << '\n';
            //myfile << data_in(0);

            if (control) {
                // std::cout << "Send" << std::endl;
                cosimul_interface.SendData(time, data_out);  // --> to Simulink
                // std::cout << "Receive" << std::endl;
                cosimul_interface.ReceiveData(histime, data_in);  // <-- from Simulink
                cart.updateBodyForce(-data_in(0), time);
                //std::cout << "--- time: " << time << std::endl;
            }

        }
        myfile.close();

    }
    catch (std::exception exception) {
        std::cerr << " ERRROR with socket system:\n" << exception.what() << std::endl;

    }
    return 0;
}
