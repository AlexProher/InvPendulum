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


#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/utils/ChSocketCommunication.h"
#include "MyCart.h"

#include <fstream>

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::utils;
using namespace rapidjson;

void ReadFileJSON(const std::string& filename, Document& d) {
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        std::cerr << "ERROR: Could not open JSON file: " << filename << std::endl;
    }
    else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            std::cerr << "ERROR: Invalid JSON file: " << filename << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {

    Document config;
    ReadFileJSON("../../sourceFiles/configuration.json", config);
    assert(config.HasMember("Position"));
    //config.ParseStream(isw);

    bool control = config["Control"].GetBool();

    try {

        ChSystemSMC sys;
        ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;
        sys.SetCollisionSystemType(collision_type);
        auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
        sys.SetCollisionSystem(collsys);

        sys.SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
        // Create a Chrono physical system
        MyCart cart(config);
        cart.addCartToSys(sys);

        // 1 - Create a floor that is fixed (that is used also to represent the absolute reference)

        vehicle::SCMTerrain mterrain(&sys);
        mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));
        double length = 4;
        double width = 10;
        double mesh_resolution = 0.05;
        mterrain.Initialize(width, length, mesh_resolution);
        mterrain.SetSoilParameters(
            0.2e6,  // Bekker Kphi
            0,      // Bekker Kc
            1.1,    // Bekker n exponent
            0,      // Mohr cohesive limit (Pa)
            30,     // Mohr friction limit (degrees)
            0.01,   // Janosi shear coefficient (m)
            4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
            3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
        mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 30000.2);
        mterrain.SetMeshWireframe(true);

        sys.SetGravitationalAcceleration(ChVector3d(0, -9.8f, 0));
        //sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        //sys.GetSolver()->AsIterative()->SetMaxIterations(20);

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
                cart.updateMotorTorque ( data_in(0)*config["Wheel"]["radius"].GetDouble()/4);
                //cart.updateBodyForce(-data_in(0), time);
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
