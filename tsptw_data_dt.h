#ifndef LOCALSOLVER_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H
#define LOCALSOLVER_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H

#include <ostream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <inttypes.h>
#include <fstream>
#include <math.h>
#include <string>

#include "localsolver_vrp.pb.h"

#define CUSTOM_MAX_INT (int) pow(2,30)

enum ShiftPref { ForceStart = 2, ForceEnd = 1, MinimizeSpan = 0 };

namespace std {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(string filename) {
    LoadInstance(filename);
  }
  void LoadInstance(const string & filename);

  vector<int> Demands() const {
    return Demands_;
  }

  vector<int> Durations() const {
    return Durations_;
  }

  vector<int> CapaVecs() const {
    return CapaVec_;
  }

  int SizeMissions() const {
    return size_missions_;
  }

  int NbVehicles() const {
    return nbVecs_;
  }

  vector<vector<float>> Matrice() const {
    return matrice_;
  }

  vector<float> DistWarehouse() const {
    return distWare;
  }

  vector<float> DistWarehouseReturn() const {
    return distWare2;
  }

  vector<int> TimeWindowStarts1() const {
    return timewindow_start_1 ;
  }

  vector<int> TimeWindowEnds1() const {
    return timewindow_end_1 ;
  }

  vector<int> TimeWindowStarts2() const {
    return timewindow_start_2 ;
  }

  vector<int> TimeWindowEnds2() const {
    return timewindow_end_2 ;
  }

  // vector<vector<int>> TimeWindowStarts() const {
  //   return timewindow_start_ ;
  // }

  // vector<vector<int>> TimeWindowEnds() const {
  //   return timewindow_end_ ;
  // }

  vector<vector<int>> IndiceMultipleTW() const {
    return indiceMultipleTW_;
  }

  int SizeMissionsMultipleTW() const {
    return size_missions_multipleTW_;
  }

  vector<int> TwStartCar() const {
    return tw_start_car_;
  }

  vector<int> TwEndCar() const {
    return tw_end_car_;
  }

  vector<int> Start_index() const {
    return start_index_;
  }

  vector<int> End_index() const {
    return end_index_;
  }

private:
  void ProcessNewLine(char* const line);


  int size_missions_;
  int size_missions_multipleTW_;
  int nbVecs_;
  vector<int> CapaVec_;
  vector<int> Demands_;
  vector<int> Durations_;
  vector<int> tw_start_car_;
  vector<int> tw_end_car_;
  // vector<vector<int>> timewindow_start_;
  // vector<vector<int>> timewindow_end_;
  vector<int> timewindow_start_1;
  vector<int> timewindow_end_1;
  vector<int> timewindow_start_2;
  vector<int> timewindow_end_2;
  vector<int> start_index_;
  vector<int> end_index_;
  vector<vector<float>> matrice_;
  vector<float> distWare;
  vector<float> distWare2;
  vector<vector<int>> indiceMultipleTW_;
  vector<int> indice_;
};

void TSPTWDataDT::LoadInstance(const string & filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  localsolver_vrp::Problem problem;

  {
    fstream input(filename, ios::in | ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      cout << "Failed to parse pbf." << endl;
    }
  }


  int nbService=0;
  for (const localsolver_vrp::Service& service: problem.services()) {
    nbService++;
  }
  size_missions_ = nbService;
  size_missions_multipleTW_ = size_missions_;

  int nbVehicle = 0;
  for (const localsolver_vrp::Vehicle& vehicle: problem.vehicles()) {
    nbVehicle +=1;
  }
  nbVecs_=nbVehicle;

  int quant=0;
  int j=0;
  int tws = 0;
  vector<int> start;
  vector<int> end;
  for (const localsolver_vrp::Service& service: problem.services()) {
    j+=1;
    Durations_.push_back(service.duration());
    for (const float& quantity: service.quantities()) {
      Demands_.push_back(quantity);
    }

    if (service.time_windows().size() != 0 && service.time_windows().size() != 1){
      for (const localsolver_vrp::TimeWindow& tw: service.time_windows()) {
        start.push_back(tw.start());
        if (tw.end() == 0) {
          end.push_back(5000000);  
        } else {
          end.push_back(tw.end());
        }
      }

      timewindow_start_1.push_back(start[0]);
      timewindow_end_1.push_back(end[0]);
      timewindow_start_2.push_back(start[1]);
      timewindow_end_2.push_back(end[1]);

      
      // }
      start.clear();
      end.clear();

    } else if (service.time_windows().size() == 1) {
      for (const localsolver_vrp::TimeWindow& tw: service.time_windows()) {
        start.push_back(tw.start());
        if (tw.end() == 0) {
          end.push_back(5000000);  
        } else {
          end.push_back(tw.end());
        }
      }
      start.push_back(-2);
      end.push_back(-1);
      timewindow_start_1.push_back(start[0]);
      timewindow_end_1.push_back(end[0]);
      timewindow_start_2.push_back(start[1]);
      timewindow_end_2.push_back(end[1]);
      start.clear();
      end.clear();
    }
    else {
      for (int i=0; i<2; i++) {
        if (i == 0) {
          start.push_back(0);
          end.push_back(5000000);
        } else {
          start.push_back(-2);
          end.push_back(-1);
        }
        
      }
      timewindow_start_1.push_back(start[0]);
      timewindow_end_1.push_back(end[0]);
      timewindow_start_2.push_back(start[1]);
      timewindow_end_2.push_back(end[1]);
      start.clear();
      end.clear();
    }
  }


  for (const localsolver_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const localsolver_vrp::Capacity& capacity: vehicle.capacities()) {
      CapaVec_.push_back(capacity.limit()/1000);
    }
    tw_start_car_.push_back(vehicle.time_window().start());
    tw_end_car_.push_back(vehicle.time_window().end());
    start_index_.push_back(vehicle.start_index());
    end_index_.push_back(vehicle.end_index());
  }


  for (const localsolver_vrp::Matrix& matrix: problem.matrices()) {
    int size_matrix5 = sqrt(matrix.time().size());
    if (size_matrix5 == size_missions_ + 2) {
      int size_matrix = sqrt(matrix.time().size());
      for (int i=0; i<size_matrix; i++) {
        vector<float> tab;
        if (i==size_matrix-2) {
          for (int j=0; j<size_matrix-2; j++) {
            distWare.push_back(static_cast<float>(matrix.time(i * (size_matrix) + j)));
          }
        } else if (i==size_matrix-1) {
          for (int j=0; j<size_matrix-2; j++) {
            distWare2.push_back(static_cast<float>(matrix.time(i * (size_matrix) + j)));
          }
        } else {
          for (int j=0; j<size_matrix-2; j++) {
            tab.push_back(static_cast<float>(matrix.time(i * (size_matrix) + j)));
          }  
          matrice_.push_back(tab);
        }
      }
    } else {
      int size_matrix = sqrt(matrix.time().size()) - (CapaVec_.size()-1)*2;
      for (int i=0; i<size_matrix; i++) {
        vector<float> tab;
        if (i==size_matrix-2) {
          for (int j=0; j<size_matrix-2; j++) {
            distWare.push_back(static_cast<float>(matrix.time(i * (size_matrix + (CapaVec_.size()-1)*2) + j)));
          }
        } else if (i==size_matrix-1) {
          for (int j=0; j<size_matrix-2; j++) {
            distWare2.push_back(static_cast<float>(matrix.time(i * (size_matrix + (CapaVec_.size()-1)*2) + j)));
          }
        } else {
          for (int j=0; j<size_matrix-2; j++) {
            tab.push_back(static_cast<float>(matrix.time(i * (size_matrix + (CapaVec_.size()-1)*2) + j)));
          }  
          matrice_.push_back(tab);
        }
      }
    }
  }
}

}  //  namespace operations_research

#endif //  LOCALSOLVER_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
