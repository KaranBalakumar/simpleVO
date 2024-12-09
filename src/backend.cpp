#include "myVO/backend.h"
#include "myVO/algorithm.h"
#include "myVO/feature.h"
#include "myVO/g2o_types.h"
#include "myVO/map.h"
#include "myVO/mappoint.h"

namespace myVO
{
    
Backend::Backend(){
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap(){
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop(){
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop(){
    while(backend_running_.load()){
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        // the backend only optimizes the activated frames and landmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks){
    //setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(
                                                            std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Pose vertex, using Keyframe id
    std::map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;
    for(auto &keyframe : keyframes){
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose(); // camera vertex pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if(kf->keyframe_id_ > max_kf_id){
            max_kf_id = kf->keyframe_id_;
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    // Landmark vertex, indexed by landmark id
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991; // robust kernel threshold
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for(auto &landmark: landmarks){
        if(landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        for(auto &obs : observations){
            if(obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if(feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

            auto frame = feat->frame_.lock();
            EdgeProjection *edge = nullptr;
            if(feat->is_on_left_image_){
                edge = new EdgeProjection(K, left_ext);
            }else{
                edge = new EdgeProjection(K, right_ext);
            }

            // If the landmark has not been added to optimization yet, add a new vertex
            if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end()){
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});void writePos();
                optimizer.addVertex(v);
            }

            if(vertices.find(frame->keyframe_id_) != vertices.end() && vertices_landmarks.find(landmark_id) != vertices_landmarks.end()){
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }else{
                delete edge;
            }
        }   
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while(iteration < 5){
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshol;d
        for(auto &ef : edges_and_features){
            if (ef.first->chi2() > chi2_th){
                cnt_outlier++;
            }else{
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if(inlier_ratio > 0.5){
            break;
        }else{
            chi2_th *= 2;
            iteration++;
        }
    }

    for(auto &ef : edges_and_features){
        if(ef.first->chi2() > chi2_th){
            ef.second->is_outlier_ = true;
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        }else{
            ef.second->is_outlier_ = false;
        }
    }
    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier;

    // Set ose and landmark position
    for(auto &v : vertices){
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for(auto &v : vertices_landmarks){
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}

void Backend::writePos(){
    Map::KeyframesType kfs = map_->GetAllKeyFrames();
    std::string filename = "/home/karan/simpleVO/result/pose.txt";

    // Open the file in write mode
    std::ofstream outFile(filename);

    // Check if the file was opened successfully
    if (!outFile) {
        std::cerr << "Error: Could not open the file " << filename << " for writing." << std::endl;
    }

    // Write each entry in the dataset to the file
    for(const auto &keyframe: kfs){
        auto kf = keyframe.second;
        auto pos = kf->Pose().inverse();
        outFile << pos.translation()[0] << " " << pos.translation()[1] << " " << pos.translation()[2] << "\n";
    }

    // Close the file
    outFile.close();

    std::cout << "Data has been written to " << filename << std::endl;
}

} // namespace myVO
