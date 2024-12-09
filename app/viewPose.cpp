#include <iostream>
#include <fstream>
using namespace std;

#include <pangolin/pangolin.h>
#include <unistd.h>
#include <eigen3/Eigen/Core>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Trajectorytype;

string estimated_file = "/home/karan/simpleVO/result/pose.txt";

void DrawTrajectory(const Trajectorytype &et);
Trajectorytype ReadTrajectory(const string &path);

int main(int argc, char **argv){
    Trajectorytype et = ReadTrajectory(estimated_file);

    // Plot both the trajectories
    DrawTrajectory(et);
}

Trajectorytype ReadTrajectory(const string &path){
    ifstream fin(path);
    Trajectorytype poses;

    if(!fin.is_open()){
        cerr << "Error opening the file" << endl;
        return poses;
    }

    while(!fin.eof()){
        double tx, ty, tz;
        fin >> tx >> ty >> tz;
        // Sophus::SE3d p1(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
        Eigen::Vector3d p1(tx, ty, tz);
        poses.push_back(p1);
    }
    return poses; 
}

void DrawTrajectory(const Trajectorytype &et){
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768); // window size
    glEnable(GL_DEPTH_TEST); // to remove hidden surface by enabling depth-buffer
    glEnable(GL_BLEND); // enables blending which could be used to implement transperency within objects
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // func to assign values to C_source  
                                                     // and C_destinationin blend equation

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    ); // setting up camera state by passing projection matrix and 
    // ModelViewLookAt(where camera is?, where object is?, where camera is looking?)

    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam)); // similarly setting up display cam parameters

    while(pangolin::ShouldQuit() == false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < et.size() - 1; i++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = et[i], p2 = et[i + 1];
            glVertex3d(p1.x() , p1.y(), p1.z());
            glVertex3d(p2.x() , p2.y(), p2.z());
            glEnd();
        }

        pangolin::FinishFrame();

        usleep(5000);   // sleep 5 ms    
    }

}