#include <bits/stdc++.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace gtsam;

int main(){
    NonlinearFactorGraph graph;
    Values initialEstimate;

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
            initialEstimate.insert( Symbol('x', id), Pose2(x, y, theta));
        }
        else if( tag=="EDGE_SE2"){
            ss>> id1>> id2>> dx>> dy>> dtheta >> l11>> l12>> l13>> l22>> l23>> l33;
            Matrix3 information;
            information<< l11,l12,l13,
                          l12,l22,l23,
                          l13,l23,l33;
            auto nmodel= noiseModel::Gaussian::Information(information);
            Pose2 measurement(dx, dy, dtheta);
            graph.add( BetweenFactor<Pose2>(Symbol('x',id1), Symbol('x',id2), measurement, nmodel));
        }
    }
    file.close();

    Vector3 sigmas(1e-3, 1e-3, 1e-3); 
    auto priorNoise = noiseModel::Diagonal::Sigmas(sigmas);
    graph.add(PriorFactor<Pose2>(Symbol('x', 0),initialEstimate.at<Pose2>(Symbol('x', 0)),priorNoise));

    cout << "vertices: " << initialEstimate.size() << "\n" << "factors:  " << graph.size() << "\n" << "initial error: " << graph.error(initialEstimate) << "\n";

    LevenbergMarquardtParams params;
    params.setLinearSolverType("MULTIFRONTAL_QR");   
    params.setVerbosityLM("SUMMARY");
    params.setMaxIterations(1000);
    params.setRelativeErrorTol(0);
    params.setAbsoluteErrorTol(0);
    params.setlambdaUpperBound(1e12);

    Values result = LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

    ofstream initialFile("../results/initialFile.csv");
    ofstream output("../results/optimizedFile_batch_SLAM.csv");

    initialFile<< "id,x,y,theta\n";
    output<< "id,x,y,theta\n";

    for(auto const& keyVal:initialEstimate){
        Key key=keyVal.key;
        Symbol symbol(key);
        Pose2 initialPose = initialEstimate.at<Pose2>(key);
        Pose2 optimizedPose = result.at<Pose2>(key);

        initialFile<< symbol.index() <<","<< initialPose.x()<<","<< initialPose.y() <<","<< initialPose.theta()<< "\n";
        output<< symbol.index() <<","<< optimizedPose.x() <<","<< optimizedPose.y() <<","<< optimizedPose.theta()<< "\n";
    }
    initialFile.close();
    output.close();
    cout<< "final error:" << graph.error(result) << endl;

}
