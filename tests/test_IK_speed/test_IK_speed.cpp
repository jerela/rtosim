#include "rtosim/rtosim.h"
#include <OpenSim/OpenSim.h>
#include <iostream>
#include <time.h>
#include <fstream>

std::string runRtosimInverseKinematics(std::string dataDir, std::string ikSetupFilename, unsigned nThreads) {

	std::cout << "PREPARING TO RUN RTOSIM IK" << std::endl;

	//Extract all the necessary filenames from the setup file
	OpenSim::InverseKinematicsTool ikt(dataDir + ikSetupFilename);
	std::string markerDataFilename = dataDir + ikt.getMarkerDataFileName();
	std::string modelFilename = dataDir + ikt.getPropertyByName("model_file").getValue<std::string>();
	OpenSim::IKTaskSet taskSet = ikt.getIKTaskSet();
	double accuracy = ikt.getPropertyByName("accuracy").getValue<double>();
	double constraintWeight = ikt.getPropertyByName("constraint_weight").getValue<SimTK::Real>();
	// Create the data queues
	rtosim::MarkerSetQueue markerSetQueue;
	rtosim::GeneralisedCoordinatesQueue generalisedCoordinatesQueue;

	//Create the synchronisation latches
	rtb::Concurrency::Latch doneWithSubscription, doneWithExecution;

	//Read the data from a trc file and move it to the appropriate `Queue`
	rtosim::MarkersFromTrc markerDataProducer(
		markerSetQueue,
		doneWithSubscription,
		doneWithExecution,
		modelFilename,
		markerDataFilename,
		false);

	//Read the data from `markerSetQueue` and solves inverse kinematics
	rtosim::QueueToInverseKinematics ik(
		markerSetQueue,
		generalisedCoordinatesQueue,
		doneWithSubscription,
		doneWithExecution,
		modelFilename,
		nThreads,
		taskSet,
		accuracy,
		constraintWeight);

	// Save the data to file
	std::string outputFilename("rtosim_ik_j" + std::to_string(nThreads));
	std::vector<std::string> columnLabels = rtosim::getCoordinateNamesFromModel(modelFilename);
	rtosim::QueueToFileLogger<rtosim::GeneralisedCoordinatesData> coordinateLogger(
		generalisedCoordinatesQueue,
		doneWithSubscription,
		doneWithExecution,
		columnLabels,
		dataDir, outputFilename, "mot");
	coordinateLogger.setConvertFromRadToDeg();
	doneWithSubscription.setCount(3);
	doneWithExecution.setCount(3);

	rtosim::QueuesSync::launchThreads(markerDataProducer, ik, coordinateLogger);
	std::cout << "RTOSIM IK COMPLETED" << std::endl;
	return outputFilename + ".mot";
}

std::string runOpenSimInverseKinematics(std::string dataDir, std::string ikSetupFilename) {

	std::cout << "PREPARING TO RUN OPENSIM IK" << std::endl;
	//Extract all the necessary filenames from the setup file
	OpenSim::InverseKinematicsTool ikt(dataDir + ikSetupFilename);
	std::string modelFileName = dataDir + ikt.getPropertyByName("model_file").getValue<std::string>();
	OpenSim::IKTaskSet taskSet = ikt.getIKTaskSet();
	std::string markerDataFilename = dataDir + ikt.getMarkerDataFileName();

	// Mittaa aikaa, joka menee IK:n laskemiseen OpenSimin API:n avulla
	// Mittaa aikaa, joka menee IK:n laskemiseen reaaliajassa 1/2/4/8 säikeen avulla
	// Mittaa aikaa, joka menee IK:n laskemiseen filtterin cutoff frequencyn arvoilla 8/16/32

	//std::cout << "SETTING MODEL " << modelFileName << std::endl;
	auto model = OpenSim::Model(modelFileName);
	model.initSystem();
	ikt.setModel(model);
	//ikt.setModel(OpenSim::Model(modelFileName));
	ikt.setName("OpenSim Inverse Kinematics");
	//ikt.setMarkerDataFileName(ikt.getMarkerDataFileName());
	ikt.setMarkerDataFileName(markerDataFilename);
	//ikt.setStartTime(0);
	//ikt.setEndTime(10);


	// Save the data to file
	std::string outputFileName("osim_ik.mot");
	ikt.setOutputMotionFileName(outputFileName);
	std::cout << "PREPARING TO RUN IK" << std::endl;
	ikt.run();
	std::cout << "OPENSIM IK COMPLETED" << std::endl;
	return outputFileName;
}

void saveDurationsToFile(std::vector <int> osim, std::vector <int> rtosim, std::vector <int> threads, std::string dir) {
	std::ofstream outFile(dir+"IK_durations.txt"); // create file into the data directory
	for (int m = 0; m < osim.size(); m++) { // iterate through all elements in the duration vector
		outFile << "number of threads = " << std::to_string(threads.at(m)) << std::endl;
		outFile << "   rtosim: " << std::to_string(rtosim.at(m)) << " ms\n";
		outFile << "   opensim: " << std::to_string(osim.at(m)) << " ms\n";
	}
}

int main() {
	std::cout << "About to begin!" << std::endl;
	clock_t startTimeOSIM, startTimeRTOSIM;
	int durationRTOSIM, durationOSIM;
	try {
		std::string dataDir = std::string(BASE_DIR) + "/data/InverseKinematics/";
		//Test rtosim IK using a variable number of threads
		std::vector <int> number_of_threads = { 1, 2, 3, 4, 6, 8, 12, 16 }; // ideally from 1 to the maximum number of CPU cores
		std::vector <int> durations_OSIM, durations_RTOSIM;
		unsigned j;
		for (unsigned k = 0; k < number_of_threads.size(); k++) { // iterate through the elements of number_of_threads vector
			j = number_of_threads.at(k);
			std::cout << "j = " + std::to_string(j) << std::endl;
			// RTOSIM
			startTimeRTOSIM = clock(); // get time right before running the RunRtosimInverseKinematics function
			std::string rtosimResultsFilename = runRtosimInverseKinematics(dataDir, "subject01_Setup_InverseKinematics.xml", j);
			durationRTOSIM = 1.e3 * (clock() - startTimeRTOSIM) / CLOCKS_PER_SEC; // get the time it took to run the function
			std::cout << "routine time for j=" << std::to_string(j) << " was " << std::to_string(durationRTOSIM) << "ms\n" << std::endl;
			// OPENSIM
			startTimeOSIM = clock();
			std::string OpenSimResultsFileName = runOpenSimInverseKinematics(dataDir, "subject01_Setup_InverseKinematics.xml");
			durationOSIM = 1.e3 * (clock() - startTimeOSIM) / CLOCKS_PER_SEC;
			std::cout << "routine time for OpenSim IK was " << std::to_string(durationOSIM) << "ms\n" << std::endl;
			durations_OSIM.push_back(durationOSIM);
			durations_RTOSIM.push_back(durationRTOSIM);
		}
		saveDurationsToFile(durations_OSIM, durations_RTOSIM, number_of_threads, dataDir); // save the durations in milliseconds for both rtosim and opensim IK
	}
	catch (const OpenSim::Exception & e) {
		e.print(std::cerr);
		return 1;
	}
	std::cout << "Done" << std::endl;
	return 0;
}