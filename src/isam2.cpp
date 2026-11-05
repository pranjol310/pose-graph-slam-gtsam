#include <bits/stdc++.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Eigenvalues>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;

struct Edge{
    int id1, id2;
    Pose2 measurement;
    Matrix3 information;
};
int main(){
    map<int, Pose2> vertices;
    vector<Edge> edges;

    ifstream file("../data/input_INTEL_g2o.g2o");
    if(!file.is_open()){
        cerr<<"Error: Unable to open file"<<endl;
        return 1;
    }
    string line;
    int id, id1, id2;
    double x,y,theta;
    double dx, dy, dtheta, l11, l12, l13, l22, l23, l33;
    while(getline(file,line)){
        stringstream ss(line);
        string tag;
        ss>> tag;

        if( tag=="VERTEX_SE2"){
            ss>> id>> x>> y>> theta;
            Pose2 pose(x,y,theta);
            vertices[id]=pose;
        }
        else if( tag=="EDGE_SE2"){
            ss>> id1>> id2>> dx>> dy>> dtheta >> l11>> l12>> l13>> l22>> l23>> l33;
            Matrix3 information;
            information<< l11,l12,l13,
                          l12,l22,l23,
                          l13,l23,l33;
            Pose2 measurement(dx, dy, dtheta);
            edges.push_back({id1, id2, measurement, information});

            double maxEntry = information.cwiseAbs().maxCoeff();
            if(maxEntry > 1e8)
            {   
                cout << "Large information value found!" << endl;
                cout << "Edge: " << id1 << " -> " << id2 << endl;
                cout << information << endl;
            } 
            
            cout << "\nEdge: " << id1<< " -> " << id2 << endl;
            cout << "Information matrix:\n"<< information << endl;

            Eigen::SelfAdjointEigenSolver<Matrix3> solver(information);

            cout << "Eigenvalues: "<< solver.eigenvalues().transpose() << endl;
        }
    }
    file.close();

    map<int,vector<Edge>> edgesByArrival;
    for (const auto& edge:edges) {
        int arrival=max(edge.id1, edge.id2);
        edgesByArrival[arrival].push_back(edge);
    }

    Vector3 Sigmas(1e-3, 1e-3, 1e-3);
    auto priorNoise=noiseModel::Diagonal::Sigmas(Sigmas);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.factorization = ISAM2Params::QR;
    ISAM2 isam(parameters);
    int c=5;  // Number of new vertices per update
    auto vertexIt=vertices.begin();
    auto edgeIt=edgesByArrival.begin();
    while(vertexIt!=vertices.end()){
        NonlinearFactorGraph newFactors;
        Values newValues;
        int count=0;
        while(vertexIt!=vertices.end() && count<c){
            int id=vertexIt->first;
            Pose2 pose=vertexIt->second;
            newValues.insert(Symbol('x',id),pose);
            if(id==vertices.begin()->first){
                newFactors.add(PriorFactor<Pose2>(Symbol('x',id), pose, priorNoise));
            }
            vertexIt++;
            count++;
        }
        int lastProcessedID=prev(vertexIt)->first;
        while(edgeIt!=edgesByArrival.end() && edgeIt->first <= lastProcessedID){
            for (const auto& edge:edgeIt->second) {
                auto edgeNoise=noiseModel::Gaussian::Information(edge.information);
                newFactors.add(BetweenFactor<Pose2>(Symbol('x', edge.id1),Symbol('x', edge.id2),edge.measurement,edgeNoise));
            }    
            edgeIt++;
        }
        isam.update(newFactors, newValues);
        Values estimate = isam.calculateEstimate();
        cout<<"Processed "<< count<<" vertices in this update\n";
    }

    Values result=isam.calculateEstimate();
    ofstream output("../results/optimizedFile_isam2.csv");

    output << "id,x,y,theta\n";
    for(const auto& vertex:vertices){
        int id=vertex.first;
        Pose2 pose=result.at<Pose2>(Symbol('x', id));
        output<<id<<","<<pose.x()<<","<<pose.y()<<","<<pose.theta()<<"\n";
    }
    output.close();
}
 