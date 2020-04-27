/* -------------------------------------------------------------------------- *
 * Copyright (c) 2010-2016 C. Pizzolato, M. Reggiani                          *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at:                                   *
 * http://www.apache.org/licenses/LICENSE-2.0                                 *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "rtosim/rtosim.h"
using namespace rtosim;

#include <iostream>
#include <string>
#include <Simbody.h>


int main() {

    std::string dataDir = std::string(BASE_DIR) + "/data/InverseKinematics/";
    std::string osimModelFileName = dataDir + "fullbody.osim";
    std::string ikTaskFileName(dataDir + "fullbody_tasks.xml");

    std::string hostname("127.0.0.1:801");
    unsigned nThreads(8); // amount of threads to use in multithreading
    double solverAccuracy(1e-5); // accuracy of the solved kinematics
    std::string resultDir(dataDir + "test_IK_from_Nexus_output");
    bool showVisualiser(false);

    resultDir = rtosim::FileSystem::getAbsolutePath(resultDir);
    rtosim::FileSystem::createDirectory(resultDir);
    std::string stopWatchResultDir(resultDir);

    //define the shared buffers
    MarkerSetQueue markerSetQueue;
    GeneralisedCoordinatesQueue generalisedCoordinatesQueue;//, filteredGeneralisedCoordinatesQueue;

    //define the barriers
    rtb::Concurrency::Latch doneWithSubscriptions;
    rtb::Concurrency::Latch doneWithExecutions;
    FlowControl runCondition(true);

    auto coordNames = getCoordinateNamesFromModel(osimModelFileName);
    auto modelMarkerNames = getMarkerNamesFromModel(osimModelFileName);

    //define the threads
    DataFromNexus dataFromNexus(
        markerSetQueue,
        doneWithSubscriptions,
        doneWithExecutions,
        runCondition,
        modelMarkerNames,
        hostname);

    std::cout << "Saving marker coordinates from Vicon into markers.trc\n";
    QueueToFileLogger<MarkerSetData> markerLogger(
        markerSetQueue,
        doneWithSubscriptions,
        doneWithExecutions,
        modelMarkerNames,
        resultDir, "markers", "trc");

    std::cout << "Initiating inverse kinematics\n";
    //read from markerSetQueue, calculate IK, and save results in generalisedCoordinatesQueue
    QueueToInverseKinematics inverseKinematics(
        markerSetQueue,
        generalisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecutions,
        osimModelFileName,
        nThreads, ikTaskFileName, solverAccuracy);

    std::cout << "Saving coordinates to file\n";

    //read from generalisedCoordinatesQueue and save to file
    rtosim::QueueToFileLogger<GeneralisedCoordinatesData> rawIkLogger(
        generalisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecutions,
        coordNames,
        resultDir, "motion", "sto");

    //read the frames from generalisedCoordinatesQueue and calculates some stats
    rtosim::FrameCounter<GeneralisedCoordinatesQueue> ikFrameCounter(
        generalisedCoordinatesQueue,
        "time-ik-throughput");

    rtosim::TimeDifference<
        MarkerSetQueue,
        GeneralisedCoordinatesQueue> ikTimeDifference(
            markerSetQueue,
            generalisedCoordinatesQueue,
            doneWithSubscriptions,
            doneWithExecutions);

    auto trigger([&runCondition]() {
        bool run(true);
        std::cin.ignore(); // enables pressing enter to end program
        runCondition.setRunCondition(false);
        std::cout << "Run condition has been set to false. Inverse kinematics should now terminate.\n";
    });
    
    doneWithSubscriptions.setCount(5); // the number in setCount must match the amount they are called in rtosim functions before this line
    doneWithExecutions.setCount(5);

    //launch, execute, and join all the threads
    //all the multithreading is in this function
    if (showVisualiser) {
        StateVisualiser visualiser(generalisedCoordinatesQueue, osimModelFileName);
        QueuesSync::launchThreads(
            dataFromNexus,
            markerLogger,
            inverseKinematics,
            rawIkLogger,
            ikTimeDifference,
            ikFrameCounter,
            trigger,
            visualiser
            );
    }
    else {
        QueuesSync::launchThreads(
            dataFromNexus,
            markerLogger,
            inverseKinematics,
            rawIkLogger,
            ikTimeDifference,
            ikFrameCounter,
            trigger
            );
    }
    //multithreaded part is over, all threads are joined

    std::cout << "Threads have been joined.\n";

    //get execution time infos
    auto stopWatches = inverseKinematics.getProcessingTimes();

    rtosim::StopWatch combinedSW("time-ikparallel-processing");
    for (auto& s : stopWatches)
        combinedSW += s;

    combinedSW.print(stopWatchResultDir);
    ikFrameCounter.getProcessingTimes().print(stopWatchResultDir);

    ikTimeDifference.getWallClockDifference().print(stopWatchResultDir + "/time-markerqueue-to-jointangles.txt");
    dataFromNexus.printLatencyData(stopWatchResultDir + "/time-latency-nexus.txt");

    std::cout << "All done! Closing program.\n";

    return 0;
}
