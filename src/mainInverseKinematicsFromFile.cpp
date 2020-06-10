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
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <Simbody.h>

void printAuthors() {

    cout << "Real-time OpenSim inverse kinematics" << endl;
    cout << "Authors: Claudio Pizzolato <c.pizzolato@griffith.edu.au>" << endl;
    cout << "         Monica Reggiani <monica.reggiani@unipd.it>" << endl << endl;
}

void printHelp() {

    printAuthors();

    auto progName(SimTK::Pathname::getThisExecutablePath());
    bool isAbsolute;
    string dir, filename, ext;
    SimTK::Pathname::deconstructPathname(progName, isAbsolute, dir, filename, ext);

    cout << "Option              Argument         Action / Notes\n";
    cout << "------              --------         --------------\n";
    cout << "-h                                   Print the command-line options for " << filename << ".\n";
    cout << "--model             ModelFilename    Specify the name of the osim model file for the investigation.\n";
    cout << "--trc               TrcFilename      Specify the name of the trc file to be used.\n";
    cout << "--task-set          TaskSetFilename  Specify the name of the XML TaskSet file containing the marker weights to be used.\n";
    cout << "--fc                CutoffFrequency  Specify the name of lowpass cutoff frequency to filter IK data.\n";
    cout << "-j                  IK threads       Specify the number of IK threads to be used.\n";
    cout << "-a                  Accuracy         Specify the IK solver accuracy.\n";
    cout << "--output            OutputDir        Specify the output directory.\n";
    cout << "--push-frequency    PushFrequency    Specify the frequency to which the trajectories are read from the storage file.\n";
    cout << "-v                                   Show visualiser.\n";
}

int main(int argc, char* argv[]) {

    ProgramOptionsParser po(argc, argv);
    if (po.exists("-h") || po.empty()) {
        printHelp();
        exit(EXIT_SUCCESS);
    }

    string osimModelFilename;
    if (po.exists("--model"))
        osimModelFilename = po.getParameter("--model");
    else {
        printHelp();
        exit(EXIT_SUCCESS);
    }

    string trcTrialFilename;
    if (po.exists("--trc"))
        trcTrialFilename = po.getParameter("--trc");
    else {
        printHelp();
        exit(EXIT_SUCCESS);
    }

    string ikTaskFilename;
    if (po.exists("--task-set"))
        ikTaskFilename = po.getParameter("--task-set");
    else {
        printHelp();
        exit(EXIT_SUCCESS);
    }

    double fc(8);
    if (po.exists("--fc"))
        fc = po.getParameter<double>("--fc");

    unsigned nThreads(1);
    if (po.exists("-j"))
        nThreads = po.getParameter<unsigned>("-j");

    double solverAccuracy(1e-5);
    if (po.exists("-a"))
        nThreads = po.getParameter<double>("-a");

    string resultDir("Output");
    if (po.exists("--output"))
        resultDir = po.getParameter("--output");

    double pushFrequency(-1);
    if (po.exists("--push-frequency"))
        pushFrequency = po.getParameter<double>("--push-frequency");

    bool showVisualiser(false);
    if (po.exists("-v"))
        showVisualiser = true;

    resultDir = rtosim::FileSystem::getAbsolutePath(resultDir);
    rtosim::FileSystem::createDirectory(resultDir);
    string stopWatchResultDir(resultDir);

    cout << "116\n";

    //define the shared buffer
    rtosim::MarkerSetQueue markerSetQueue;
    rtosim::GeneralisedCoordinatesQueue generalisedCoordinatesQueue, filteredGeneralisedCoordinatesQueue;

    cout << "122\n";

    //define the barriers
    rtb::Concurrency::Latch doneWithSubscriptions;
    rtb::Concurrency::Latch doneWithExecution;

    cout << "128\n"; // t�h�n loppuu yleens�

    //define the filter
    auto coordNames = getCoordinateNamesFromModel(osimModelFilename);
    rtosim::GeneralisedCoordinatesStateSpace gcFilt(fc, coordNames.size());
    cout << coordNames[1] << endl;
    cout << coordNames[2] << endl;
    cout << "134\n"; // tyss�ys

    //define the threads
    //#read markers from file and save them in markerSetQueue
    rtosim::MarkersFromTrc markersFromTrc(
        markerSetQueue,
        doneWithSubscriptions,
        doneWithExecution,
        osimModelFilename,
        trcTrialFilename,
        false);

    cout << "146\n";

    markersFromTrc.setOutputFrequency(pushFrequency);

    cout << "150\n";

    //read from markerSetQueue, calculate IK, and save results in generalisedCoordinatesQueue
    rtosim::QueueToInverseKinematics inverseKinematics(
        markerSetQueue,
        generalisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecution,
        osimModelFilename,
        nThreads, ikTaskFilename, solverAccuracy);

    cout << "161\n";

    //read from generalisedCoordinatesQueue, filter using gcFilt,
    //and save filtered data in filteredGeneralisedCoordinatesQueue
    rtosim::QueueAdapter <
        rtosim::GeneralisedCoordinatesQueue,
        rtosim::GeneralisedCoordinatesQueue,
        rtosim::GeneralisedCoordinatesStateSpace
    > gcQueueAdaptor(
    generalisedCoordinatesQueue,
    filteredGeneralisedCoordinatesQueue,
    doneWithSubscriptions,
    doneWithExecution,
    gcFilt);

    cout << "176\n"; // t�nne asti p��sty

    //read from filteredGeneralisedCoordinatesQueue and save to file
    rtosim::QueueToFileLogger<rtosim::GeneralisedCoordinatesData> filteredIkLogger(
        filteredGeneralisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecution,
        getCoordinateNamesFromModel(osimModelFilename),
        resultDir, "filtered_ik_from_file", "sto");

    cout << "186\n"; // t�nne asti p��sty joskus harvoin

    //read from generalisedCoordinatesQueue and save to file
    rtosim::QueueToFileLogger<rtosim::GeneralisedCoordinatesData> rawIkLogger(
        generalisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecution,
        getCoordinateNamesFromModel(osimModelFilename),
        resultDir, "raw_ik_from_file", "sto");

    cout << "196\n";

    //calculate the ik throughput time
    rtosim::FrameCounter<rtosim::GeneralisedCoordinatesQueue> ikFrameCounter(
        generalisedCoordinatesQueue,
        "time-ik-throughput");

    cout << "203\n";

    //measures the time that takes every single frame to appear in two different queues
    rtosim::TimeDifference<
        rtosim::GeneralisedCoordinatesQueue,
        rtosim::GeneralisedCoordinatesQueue> gcQueueAdaptorTimeDifference(
        generalisedCoordinatesQueue,
        filteredGeneralisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecution);

    cout << "214\n";

    rtosim::TimeDifference<
        rtosim::MarkerSetQueue,
        rtosim::GeneralisedCoordinatesQueue> ikTimeDifference(
        markerSetQueue,
        generalisedCoordinatesQueue,
        doneWithSubscriptions,
        doneWithExecution);

    cout << "224\n";

    doneWithSubscriptions.setCount(7);
    doneWithExecution.setCount(7);

    cout << "229\n"; // t�nne asti p��sty tosi harvoin

    //launch, execute, and join all the threads
    //all the multithreading is in this function
    if (showVisualiser) {
        rtosim::StateVisualiser visualiser(generalisedCoordinatesQueue, osimModelFilename);
        rtosim::QueuesSync::launchThreads(
            markersFromTrc,
            inverseKinematics,
            gcQueueAdaptor,
            filteredIkLogger,
            gcQueueAdaptorTimeDifference,
            ikTimeDifference,
            rawIkLogger,
            ikFrameCounter,
            visualiser
            );
    }
    else {
        rtosim::QueuesSync::launchThreads(
            markersFromTrc,
            inverseKinematics,
            gcQueueAdaptor,
            filteredIkLogger,
            gcQueueAdaptorTimeDifference,
            ikTimeDifference,
            rawIkLogger,
            ikFrameCounter
            );
    }
    //multithreaded part is over, all threads are joined

    cout << "261\n"; // t�nne asti p��sty, ja loppuun my�s

    auto stopWatches = inverseKinematics.getProcessingTimes();
    rtosim::StopWatch combinedSW("time-ikparallel-processing");
    for (auto& s : stopWatches)
        combinedSW += s;
    combinedSW.print(stopWatchResultDir);
    ikFrameCounter.getProcessingTimes().print(stopWatchResultDir);
    ikTimeDifference.getWallClockDifference().print(stopWatchResultDir + "/time-markerqueue-to-jointangles.txt");
    gcQueueAdaptorTimeDifference.getWallClockDifference().print(stopWatchResultDir + "/time-jointangles-to-filteredjointangles.txt");
    return 0;
}