

#include <gflags/gflags.h>
#include <cmath>
#include <string>
#include <cstdlib>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include "localsolver.h"
#include "tsptw_data_dt.h"
#include "localsolver_result.pb.h"

DEFINE_string(instance_file, "", "instance file name or data");
DEFINE_string(solution_file, "", "instance file name or data");

using namespace localsolver;
using namespace std;


class Cvrptw {
public:

    // LocalSolver
    LocalSolver localsolver;

    // Number of customers
    int nbCustomers;

    // Capacity of the trucks
    vector<int> truckCapacity;
    
    // Latest allowed arrival to depot
    int maxHorizon;

    // Demand on each node
    vector<int> demands;

    // Earliest arrival on each node
    // vector<vector<int>> earliestStart;
    vector<int> earliestStart1;
    vector<int> earliestStart2;

    // Latest departure from each node
    // vector<vector<int>> latestEnd;
    vector<int> latestEnd1;
    vector<int> latestEnd2;

    // Service time on each node
    vector<int> serviceTime;

    // Distance matrix
    vector<vector<float>> distanceMatrix;

    // Distance to depot
    vector<float> distanceWarehouses;

    vector<float> distanceWarehousesReturn;

    // Number of trucks
    int nbTrucks;

    // Decision variables
    vector<LSExpression> customersSequences;

    // Are the trucks actually used
    vector<LSExpression> trucksUsed;

    // Cumulated lateness in the solution (must be 0 for the solution to be valid)
    LSExpression totalLateness;

    // Cumulated waiting time in the solution 
    LSExpression totalWaitingTime;

    // Cumulated homelateness 
    LSExpression totalHomeLateness;

    // Number of trucks used in the solution
    LSExpression nbTrucksUsed;

    // Distance travelled by all the trucks
    LSExpression totalDistance;

    // Cumulated time served 
    LSExpression totalServiceTime;

    // Start of each visit without waiting time and with twstart_car as departure
    vector<LSExpression> startTime;

    // Start of each visit with waiting time
    vector<LSExpression> startTime1;

    // End of each visit without waiting time and with twstart_car as departure
    vector<LSExpression> endTime;

    // End of each visit with waiting time
    vector<LSExpression> endTime1;

    // Vector of decision variable on the start of each vehicle
    vector<LSExpression> StartDepot;

    // Sum of all the decision variable StartDepot
    LSExpression StartDepotTotal;

    // Vector of waiting time of a truck
    vector<LSExpression> waitingTotalTruck;

    // Cumulated marge (if we compute the waiting time without decision variable)
    vector<LSExpression> marge;


    // Constructor
    Cvrptw() {
    }


    int solve(const TSPTWDataDT &data, string filename) {
            GOOGLE_PROTOBUF_VERIFY_VERSION;
            localsolver_result::Result result;

            const int size_missions = data.SizeMissions();
            const int size_missions_multipleTW = data.SizeMissionsMultipleTW();
            const int nbVehicle = data.NbVehicles();
            truckCapacity = data.CapaVecs();
            demands = data.Demands();
            serviceTime = data.Durations();
            earliestStart1 = data.TimeWindowStarts1();
            latestEnd1 = data.TimeWindowEnds1();
            earliestStart2 = data.TimeWindowStarts2();
            latestEnd2 = data.TimeWindowEnds2();
            distanceWarehouses = data.DistWarehouse();
            distanceWarehousesReturn = data.DistWarehouseReturn();
            const vector<int> tw_start_car = data.TwStartCar();
            const vector<int> tw_end_car = data.TwEndCar();
            const vector<int> start_index = data.Start_index();
            const vector<int> end_index = data.End_index();
            const vector<vector<int>> indiceMultipleTW = data.IndiceMultipleTW();

            distanceMatrix = data.Matrice();
            nbTrucks = data.TwStartCar().size();
            nbCustomers = size_missions;
            int p = 0;
            bool bouya = false;

            // Declares the optimization model.
            LSModel model = localsolver.getModel();

            // Sequence of customers visited by each truck.
            customersSequences.resize(nbTrucks);
            StartDepot.resize(nbTrucks);
            for (int k = 0; k < nbTrucks; k++) {
                customersSequences[k] = model.listVar(nbCustomers);
                StartDepot[k] = model.intVar(tw_start_car[k], tw_end_car[k]);
            }

            // All customers must be visited by  the trucks
            model.constraint(model.partition(customersSequences.begin(), customersSequences.end()));
            
            // Create demands, earliest, latest and service as arrays to be able to access it with an "at" operator
            LSExpression demandsArray = model.array(demands.begin(), demands.end());

            // LSExpression earliestArray = model.array();
            // LSExpression latestArray = model.array();
            // for (int i=0; i<2; i++) {
            //     earliestArray.addOperand(model.array(earliestStart[i].begin(), earliestStart[i].end()));
            //     latestArray.addOperand(model.array(latestEnd[i].begin(), latestEnd[i].end()));
            // }

            // LSExpression earliestArray = model.array(earliestStart.begin(), earliestStart.end());
            // LSExpression latestArray = model.array(latestEnd.begin(), latestEnd.end());

            LSExpression earliestArray1 = model.array(earliestStart1.begin(), earliestStart1.end());
            LSExpression latestArray1 = model.array(latestEnd1.begin(), latestEnd1.end());

            LSExpression earliestArray2 = model.array(earliestStart2.begin(), earliestStart2.end());
            LSExpression latestArray2 = model.array(latestEnd2.begin(), latestEnd2.end());

            
            LSExpression serviceArray = model.array(serviceTime.begin(), serviceTime.end());
            LSExpression twStartCar = model.array(tw_start_car.begin(), tw_start_car.end());
            LSExpression twEndCar = model.array(tw_end_car.begin(), tw_end_car.end());

            cout << nbCustomers << endl;
            cout << distanceMatrix.size() << endl;
            // Create distance as an array to be able to acces it with an "at" operator
            LSExpression distanceArray = model.array();
            for (int n = 0; n < nbCustomers; n++) {
                distanceArray.addOperand(model.array(distanceMatrix[n].begin(), distanceMatrix[n].end()));
            }
            
            LSExpression distanceWarehousesArray = model.array(distanceWarehouses.begin(), distanceWarehouses.end());
            LSExpression distanceWarehousesReturnArray = model.array(distanceWarehousesReturn.begin(), distanceWarehousesReturn.end());

            trucksUsed.resize(nbTrucks);
            vector<LSExpression> routeDistances(nbTrucks), endTime(nbTrucks), endTime1(nbTrucks), startTime(nbTrucks), startTime1(nbTrucks), homeLateness(nbTrucks), lateness(nbTrucks), marge(nbTrucks), mama(nbTrucks), endTime0(nbTrucks);
            vector<LSExpression> waitingArray(nbTrucks), comparaison(nbTrucks), serviceTimeServed(nbTrucks);
            vector<LSExpression> waitingTotalTruck(nbTrucks); 
            vector<LSExpression> arrivalTimeArray(nbTrucks);

            int distanceTravvelled;
            for (int k = 0; k < nbTrucks; k++) {
                LSExpression sequence = customersSequences[k];
                LSExpression c = model.count(sequence);
                
                // A truck is used if it visits at least one customer
                trucksUsed[k] = c > 0;
                maxHorizon = tw_end_car[k];
                    
                // The quantity needed in each route must not exceed the truck capacity
                LSExpression demandSelector = model.createFunction([&](LSExpression i) { return demandsArray[sequence[i]]; });
                LSExpression routeQuantity = model.sum(model.range(0, c), demandSelector);
                model.constraint(routeQuantity <= truckCapacity[k]);


                // LSExpression compare = model.createFunction([&](LSExpression i, LSExpression prev) {
                //     return model.iif(i == 0, model.iif(model.at(earliestArray2, sequence[i]) == -2, 
                //                                     model.at(earliestArray1, sequence[i]),
                //                                     model.iif(distanceWarehousesArray[sequence[0]] + StartDepot[k] > model.at(latestArray1, sequence[i]) - model.at(serviceArray, sequence[i]),
                //                                               model.at(earliestArray2, sequence[i]),
                //                                               model.at(earliestArray1, sequence[i]))), 
                //                             model.iif(model.at(earliestArray2, sequence[i]) == -2,
                //                                     model.at(earliestArray1, sequence[i]),
                //                                     model.iif(prev + model.at(distanceArray,sequence[i-1],sequence[i]) > model.at(latestArray1, sequence[i]) - model.at(serviceArray, sequence[i]),
                //                                               model.at(earliestArray2, sequence[i]),
                //                                               model.at(earliestArray1, sequence[i]))));
                // });

                // comparaison[k] = model.array(model.range(0,c), compare);


                // // Gives the end of each visit if the truck start at timewindow.start
                // LSExpression endSelector = model.createFunction([&](LSExpression i, LSExpression prev) {  
                //  return model.max(comparaison[k][i],
                //           model.iif(i == 0, 
                //                   distanceWarehousesArray[sequence[0]] + tw_start_car[k],
                //                   prev + model.at(distanceArray,sequence[i-1],sequence[i]))
                //                   ) + 
                //           model.at(serviceArray, sequence[i]); });
                
                // endTime[k] = model.array(model.range(0,c), endSelector);

                // // Gives the start of each visit if the truck start at timewindow.start
                // LSExpression startSelector = model.createFunction([&](LSExpression i, LSExpression prev) {  
                //  return model.min(comparaison[k][i],
                //           model.iif(i == 0, 
                //                   distanceWarehousesArray[sequence[0]] + tw_start_car[k],
                //                   prev + model.at(distanceArray,sequence[i-1],sequence[i]))
                //                   ); });
                
                // startTime[k] = model.array(model.range(0,c), startSelector);


                // // Compute the distance 
                // LSExpression totodist = model.createFunction([&](LSExpression i) {
                //     return model.at(distanceArray, sequence[i-1], sequence[i]);
                // });

                // LSExpression totoservice = model.createFunction([&](LSExpression i) {
                //     return  model.at(serviceArray, sequence[i]);
                // });


                // LSExpression endSelector1 = model.createFunction([&](LSExpression i, LSExpression prev) {
                //     return model.max(comparaison[k][i],
                //         model.iif(i == 0,
                //             distanceWarehousesArray[sequence[0]] + (endTime[k][c-1] - model.sum(model.range(1,c), totodist) - distanceWarehousesArray[sequence[0]] - model.sum(model.range(0,c), totoservice)),
                //             prev + model.at(distanceArray,sequence[i-1],sequence[i])));

                // });
                // endTime1[k] = model.array(model.range(0,c), endSelector1);



                // LSExpression startSelector1 = model.createFunction([&](LSExpression i, LSExpression prev) {  
                //  return model.min(comparaison[k][i],
                //           model.iif(i == 0, 
                //                   distanceWarehousesArray[sequence[0]] + (endTime1[k][c-1] - model.sum(model.range(1,c), totodist) - distanceWarehousesArray[sequence[0]] - model.sum(model.range(0,c), totoservice)),
                //                   prev + model.at(distanceArray,sequence[i-1],sequence[i]))
                //                   ); });
                
                // startTime1[k] = model.array(model.range(0,c), startSelector1);



                // LSExpression marge = model.createFunction([&](LSExpression i) {
                //     return model.iif(i == 0, 
                //         comparaison[k][i] - distanceWarehousesArray[sequence[0]] - (endTime1[k][c-1] - model.sum(model.range(1,c), totodist) - distanceWarehousesArray[sequence[0]] - model.sum(model.range(0,c), totoservice)),
                //         model.max(0, comparaison[k][i] - startTime1[k][i] ));
                // });

                // LSExpression waitingTime = model.createFunction([&](LSExpression i) {
                //     return model.iif(i == 0, 
                //             0,
                //             model.max(0, comparaison[k][i] - endTime1[k][i-1] - model.sum(model.range(0,i-1), marge) - model.at(distanceArray,sequence[i-1],sequence[i])));
                // });





                // Gives the right timewindow to compare to the actual time
                LSExpression compare = model.createFunction([&](LSExpression i, LSExpression prev) {
                    return model.iif(i == 0, model.iif(model.at(earliestArray2, sequence[i]) == -2, 
                                                    model.at(earliestArray1, sequence[i]),
                                                    model.iif(distanceWarehousesArray[sequence[0]] + StartDepot[k] > model.at(latestArray1, sequence[i]) - model.at(serviceArray, sequence[i]),
                                                              model.at(earliestArray2, sequence[i]),
                                                              model.at(earliestArray1, sequence[i]))), 
                                            model.iif(model.at(earliestArray2, sequence[i]) == -2,
                                                    model.at(earliestArray1, sequence[i]),
                                                    model.iif(prev + model.at(distanceArray,sequence[i-1],sequence[i]) > model.at(latestArray1, sequence[i]) - model.at(serviceArray, sequence[i]),
                                                              model.at(earliestArray2, sequence[i]),
                                                              model.at(earliestArray1, sequence[i]))));
                });

                comparaison[k] = model.array(model.range(0,c), compare);

                // Gives the end of each visit
                LSExpression endSelector = model.createFunction([&](LSExpression i, LSExpression prev) {
                 return model.max(comparaison[k][i],
                          model.iif(i == 0,
                                  distanceWarehousesArray[sequence[0]] + StartDepot[k],
                                  prev + model.at(distanceArray,sequence[i-1],sequence[i]))
                                  ) +
                          model.at(serviceArray, sequence[i]); });
                endTime1[k] = model.array(model.range(0,c), endSelector);

                model.constraint(model.at(twStartCar, k) <= StartDepot[k]);
                model.constraint(StartDepot[k] <= model.at(twEndCar, k));

                // Compute the waiting time based on enTime1
                LSExpression waitingTime = model.createFunction([&](LSExpression i) {
                    return model.iif(i == 0, 
                            model.iif(model.at(earliestArray2, sequence[i]) == -2,
                                model.max(0, model.at(earliestArray1, sequence[i]) - distanceWarehousesArray[sequence[0]] - StartDepot[k]),
                                model.min(model.max(0, model.at(earliestArray1, sequence[i])- distanceWarehousesArray[sequence[0]] - StartDepot[k]), model.max(0, model.at(earliestArray2, sequence[i]) - distanceWarehousesArray[sequence[0]] - StartDepot[k]))), 
                            model.iif(model.at(earliestArray2, sequence[i]) == -2,
                                model.max(0, model.at(earliestArray1, sequence[i]) - endTime1[k][i-1] - model.at(distanceArray,sequence[i-1],sequence[i])),
                                model.min(model.max(0, model.at(earliestArray1, sequence[i]) - endTime1[k][i-1] - model.at(distanceArray,sequence[i-1],sequence[i])), model.max(0, model.at(earliestArray2, sequence[i]) - endTime1[k][i-1] - model.at(distanceArray,sequence[i-1],sequence[i])))));
                });


                // Arriving home after max_horizon
                homeLateness[k] = model.iif(trucksUsed[k], 
                                model.max(0,endTime1[k][c-1] + distanceWarehousesReturnArray[sequence[c-1]] - maxHorizon),
                                0);

                // Total waiting time for truck k
                waitingTotalTruck[k] = model.sum(model.range(0,c), waitingTime);


                //completing visit after latest_end
                LSExpression lateSelector = model.createFunction([&](LSExpression i) { 
                    return model.iif(model.at(earliestArray2, sequence[i]) == -2, 
                                    model.iif(model.at(earliestArray1, sequence[i]) == 0, 0, model.max(0,endTime1[k][i] - model.at(latestArray1, sequence[i]))), //- 2*model.at(serviceArray, sequence[i])), 
                                    model.max(0, model.iif(endTime1[k][i] - model.at(latestArray2, sequence[i]) > 0, endTime1[k][i] - model.at(latestArray2, sequence[i]), model.max(0, endTime1[k][i] - model.at(latestArray1, sequence[i])))));
                });


                lateness[k] = model.sum(model.range(0,c),lateSelector);

                LSExpression distSelector = model.createFunction([&](LSExpression i) { return model.at(distanceArray, sequence[i - 1], sequence[i]); });

                // Distance done by truck k
                routeDistances[k] = model.sum(model.range(1, c), distSelector) + model.sum(model.range(1, c), waitingTime) +// lateness[k] +
                    model.iif(c > 0, distanceWarehousesArray[sequence[0]] + distanceWarehousesReturnArray[sequence[c - 1]], 0);

                LSExpression serviceServed = model.createFunction([&](LSExpression i) {
                    return model.at(serviceArray, sequence[i]);
                });

                serviceTimeServed[k] = model.sum(model.range(0,c), serviceServed);

            }

            // Start of cars
            StartDepotTotal = model.sum(StartDepot.begin(), StartDepot.end());
            // Waiting Time
            totalWaitingTime = model.sum(waitingTotalTruck.begin(), waitingTotalTruck.end());
            // Total lateness
            totalLateness = model.sum(lateness.begin(), lateness.end());
            // Total nb trucks used
            nbTrucksUsed = model.sum(trucksUsed.begin(), trucksUsed.end());
            // Total distance travelled
            totalDistance = (model.round(100*model.sum(routeDistances.begin(), routeDistances.end()))/100); //+ penalty;
            // Total serviceTime done
            totalServiceTime = model.sum(serviceTimeServed.begin(), serviceTimeServed.end());
            // Total HomeLateness
            totalHomeLateness = model.sum(homeLateness.begin(), homeLateness.end());


            // Objective: minimize the number of trucks used, then minimize the distance travelled
            model.minimize(totalLateness);
            model.minimize(totalHomeLateness);
            model.minimize(totalDistance);
            // model.minimize(totalWaitingTime);
            model.maximize(StartDepotTotal);
            model.minimize(totalServiceTime);
            
            model.close();

            // Parameterizes the solver.
            LSPhase phase = localsolver.createPhase();
            phase.setTimeLimit(300);
            // phase.setOptimizedObjective(1);
            localsolver.solve();

            for (int k = 0; k < nbTrucks; k++) {
                cout << "START " << StartDepot[k].getIntValue() << endl;
            }

            cout << "NBTRUCK " << nbTrucksUsed.getIntValue() << endl;

            cout << "LATENESS " << totalLateness.getDoubleValue() << endl;

            cout << "TEMPS DATTENTE " << totalWaitingTime.getDoubleValue() << endl;

            cout << "TEMPS RETARD HOME " << totalHomeLateness.getDoubleValue() << endl;

            cout << "Objective value : " << totalDistance.getDoubleValue() << endl;
            for (int k = 0; k < nbTrucks; k++) {
                // if (trucksUsed[k].getValue() != 1) continue;
                // Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
                // as in the data files (1 being the depot)
                LSCollection customersCollection = customersSequences[k].getCollectionValue();
                cout << "TRUCK " << k << endl;
                cout << "Il y a " << customersCollection.count() << " visite dans cette tournée !" << endl;
                for (lsint i = 0; i < customersCollection.count(); i++) {
                    // cout << customersCollection[i] + 1 << " " << demands[customersCollection[i]] << " " << earliestStart[customersCollection[i]][0] << " " << latestEnd[customersCollection[i]][0] << " " << earliestStart[customersCollection[i]][1] << " " << latestEnd[customersCollection[i]][1] << endl;
                    // cout << customersCollection[i] + 1 << " " << demands[customersCollection[i]] << " " << earliestStart[customersCollection[i]] << " " << latestEnd[customersCollection[i]] << endl;
                    cout << customersCollection[i] + 1 << endl;
                }
                cout << endl;
            }

            result.set_cost(totalDistance.getDoubleValue() + totalWaitingTime.getDoubleValue());
            result.clear_routes();
            for (int k=0; k<nbTrucks; k++) {
              localsolver_result::Route* route = result.add_routes();
              int index=1;
              int quant = 0;
              LSCollection customersCollection = customersSequences[k].getCollectionValue();
              if (quant == 0) {
                  localsolver_result::Activity* activity = route->add_activities();
                  activity->set_type("start");
                  activity->set_index(-1);
              }

              for(lsint i = 0; i < customersCollection.count(); i++) {
                localsolver_result::Activity* activity = route->add_activities();
                activity->set_type("delivery");
                activity->set_index(customersCollection[i]);
                quant += demands[customersCollection[i]];
                activity->add_quantities(quant);
              }
              localsolver_result::Activity* activity = route->add_activities();
              activity->set_type("end");
              activity->set_index(-1);
            }
            fstream output(filename, ios::out | ios::trunc | ios::binary);
            if (!result.SerializeToOstream(&output)) {
              cout << "Failed to write result." << endl;
              return -1;
            }
            output.close();
    }

    // Writes the solution in a file with the following format :
    //  - number of trucks used and total distance
    //  - for each truck the nodes visited (omitting the start/end at the depot)
    void writeSolution(const string& fileName) {
        ofstream outfile;
        outfile.exceptions(ofstream::failbit | ofstream::badbit);
        outfile.open(fileName.c_str());
        int prev;
        int tempo;


        outfile << nbTrucksUsed.getValue() << " " << totalDistance.getDoubleValue() << endl;
        for (int k = 0; k < nbTrucks; k++) {
            if (trucksUsed[k].getValue() != 1) continue;
            // Values in sequence are in [0..nbCustomers-1]. +2 is to put it back in [2..nbCustomers+1]
            // as in the data files (1 being the depot)
            LSCollection customersCollection = customersSequences[k].getCollectionValue();
            for (lsint i = 0; i < customersCollection.count(); i++) {
                outfile << customersCollection[i]  << " ";
            }
            outfile << endl;
        }
    }
};


int main(int argc, char **argv) {
    
  const char* solFile = argc > 2 ? argv[2] : NULL;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  TSPTWDataDT tsptw_data(FLAGS_instance_file);

  try {
      Cvrptw model;
      // model.readInstance(instanceFile);
      model.solve(tsptw_data, FLAGS_solution_file);
      if(solFile != NULL) model.writeSolution(solFile);
      return 0;
  } catch (const exception& e){
      cerr << "Error occurred: " << e.what() << endl;
      return 1;
  }

  gflags::ShutDownCommandLineFlags();

  return 0;
}

