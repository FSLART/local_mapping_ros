//
// Created by carlostojal on 12-01-2024.
//

#include <local_mapping_ros/post_processing/Filtering.h>

namespace t24e::local_mapper::post_processing {

    float Filtering::entropyRow(float* row, std::size_t size) {
        // calculate the entropy of the row
        float entropy = 0;
        for(int i = 0; i < size; i++) {
            float p = row[i];
            entropy += p * std::log(p);
        }
        return -entropy;
    }

    float Filtering::maxRow(float* row, std::size_t size) {
        // calculate the entropy of the row
        float max = 0;
        for(int i = 0; i < size; i++) {
            float p = row[i];
            if(p > max) {
                max = p;
            }
        }
        return max;
    }

    std::pair<float*,std::size_t> Filtering::filter(float* predictions, std::vector<int64_t> tensorDims, float entThreshold, float scoreThreshold) {
        // filtered row counter
        size_t numRows = 0;

        size_t size = tensorDims[0];
        for(size_t i = 1; i < tensorDims.size(); i++) {
            size *= tensorDims[i];
        }

        // initialize the filtered predictions tensor with the same size as the predictions tensor filled with zeros
        float* filteredPredictions = new float[size]();

        // create the thread pool
        ThreadPool pool((std::thread::hardware_concurrency()));

        // mutex to protect concurrent access to the filtered predictions tensor
        std::mutex mut;

        // condition variable to protect concurrent access to the filtered predictions tensor
        std::condition_variable cv;
        bool predictionsBeingUpdated = false;

        // iterate predictions and add jobs to the pool
        for(int i = 0; i < tensorDims[0]; i++) {

            auto job = [&predictions, &tensorDims, &filteredPredictions, &numRows, &mut, &cv, &predictionsBeingUpdated, entThreshold, scoreThreshold](size_t threadIdx){

                float* row = predictions + (threadIdx * tensorDims[1]);

                // calculate the entropy and maximum of the row
                float entropy = entropyRow(row, tensorDims[1]);
                float max = maxRow(row, tensorDims[1]);

                // verify filter conditions 
                if(entropy < entThreshold && max > scoreThreshold) {
                    std::unique_lock lk(mut);
                    cv.wait(lk, [&predictionsBeingUpdated]{
                        return !predictionsBeingUpdated;
                    });
                    std::copy(row, row + tensorDims[1], filteredPredictions + (numRows++ * tensorDims[1]));
                }
                cv.notify_one();
            };
            
            pool.queueJob(job);
        }

        // start the pool
        pool.start();

        // wait for completion
        pool.join();

        return std::make_pair(filteredPredictions, numRows);
    }

    float Filtering::calculateIoU(cnn::bounding_box_t box1, cnn::bounding_box_t box2) {
        
        // get coordinates of intersection rectangle
        int xA = std::max(box1.box.first.first, box2.box.first.first);
        int yA = std::max(box1.box.first.second, box2.box.first.second);
        int xB = std::min(box1.box.first.first + box1.box.second.first, box2.box.first.first + box2.box.second.first);
        int yB = std::min(box1.box.first.second + box1.box.second.second, box2.box.first.second + box2.box.second.second);


        // compute the area of intersection rectangle
        int interArea = std::max(0, xB - xA + 1) * std::max(0, yB - yA);

        // compute the area of the union
        int area1 = box1.box.second.first * box1.box.second.second;
        int area2 = box2.box.second.first * box2.box.second.second;
        int unionArea = area1 + area2 - interArea;

        // compute the intersection over union
        float iou = static_cast<float>(interArea) / static_cast<float>(unionArea);

        return iou;
    }

    std::vector<cnn::bounding_box_t> Filtering::nmsIoU(float* predictions, std::vector<int64_t> tensorDims, 
    float entThreshold, float scoreThreshold, float IoUThreshold, float* boundingBoxes) {

        std::vector<cnn::bounding_box_t> boundingBoxes;

        // filter the predictions based on the entropy of the prediction
        std::pair<float*,size_t> filtered;
        float* filteredPredictions = new float[tensorDims[0] * tensorDims[1]]();

        filtered = filter(predictions[0], entThreshold, scoreThreshold);
        filteredPredictions = filtered.first;

        // convert the tensor to a vector of bounding boxes
        for(int i = 0; i < filteredPredictions.size(0); i++) {
            at::Tensor row = filteredPredictions[i];
            float score = row[0].item<float>();
            if(score > scoreThreshold) {
                // get the bounding box
                cnn::bounding_box_t box;
                box.score = score;
                box.box.first.first = boundingBoxesTensor[i][1].item<int>();
                box.box.first.second = boundingBoxesTensor[i][2].item<int>();
                box.box.second.first = boundingBoxesTensor[i][3].item<int>();
                box.box.second.second = boundingBoxesTensor[i][4].item<int>();
                boundingBoxes.push_back(box);
            }
        }

        // sort the bounding boxes by score
        std::sort(boundingBoxes.begin(), boundingBoxes.end(), [](cnn::bounding_box_t& box1, cnn::bounding_box_t& box2) {
            return box1.score > box2.score;
        });

        ThreadPool pool(std::thread::hardware_concurrency());

        // mutex to protect concurrent access to the bounding boxes vector
        std::mutex mut;

        // condition variable to protect concurrent access to the bounding boxes vector
        std::condition_variable cv;
        bool boundingBoxesBeingUpdated = false;

        // perform non-maximum supression in a thread pool
        for(size_t i = 0; i < boundingBoxes.size(); i++) {
            for(size_t j = i + 1; j < boundingBoxes.size(); j++) {

                auto job = [&boundingBoxes, &i, &j, &IoUThreshold, &mut, &cv, &boundingBoxesBeingUpdated](size_t threadIdx){

                    (void)(threadIdx);

                    float iou = calculateIoU(boundingBoxes[i], boundingBoxes[j]);
                    if(iou > IoUThreshold) {
                        std::unique_lock<std::mutex> lk(mut);
                        cv.wait(lk, [&boundingBoxesBeingUpdated]{
                            return !boundingBoxesBeingUpdated;
                        });
                        boundingBoxes.erase(boundingBoxes.begin() + j);
                        j--;
                    }
                    cv.notify_one();
                };

                // at each iteration, add a job to the pool
                pool.queueJob(job);
            }
        }

        // start the pool
        pool.start();

        // wait for completion
        pool.join();
        
        return boundingBoxes;
    }
};